#!/usr/bin/env python3
import os
import math
from multiprocessing import Lock
from typing import Optional

import cv2
import numpy as np
import rospy
import tf
from dt_apriltags import Detector
from duckietown_msgs.msg import WheelEncoderStamped
from sensor_msgs.msg import CompressedImage, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from std_msgs.msg import String, Bool
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown.utils.image.ros import compressed_imgmsg_to_rgb

from ekf import EKF
from odometry_utils import delta_phi, get_odometry
from map_graph import Direction, load_graph_from_yaml
import yaml, json


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class NavigationNode(DTROS):
    """
    Owns both localization (EKF over AprilTags + wheel odometry) and global
    path planning. Localization is folded in here because it has exactly one
    consumer - navigation - so a separate node only adds message latency for
    no isolation benefit. The estimated pose is still published on
    ~/ekf_localization_node/pose (Odometry) for external observers like the
    visualizer; navigation itself reads self.ekf.q directly.
    """

    right_tick_prev: Optional[int]
    left_tick_prev: Optional[int]

    def __init__(self, node_name):
        super(NavigationNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.veh = os.environ["VEHICLE_NAME"]
        self.sim = rospy.get_param("~test_sim", False)
        self.left_wheel_mutex = Lock()
        self.right_wheel_mutex = Lock()
        self.ekf_lock = Lock()

        self.latest_img = None
        self.gt_pose = None
        self.W = self.H = self.K = self.D = self.P = None
        self.rect_camera_K = None
        self._reset_encoder_state()

        self.R = 0.0318        # wheel radius, meters
        self.baseline = 0.11   # meters, DB21

        q_0 = np.array([
            rospy.get_param("~x_0", 0.0),
            rospy.get_param("~y_0", 0.0),
            rospy.get_param("~theta_0", 0.0),
        ])
        P_0 = np.diag([
            rospy.get_param("~P_0_xx", 0.0),
            rospy.get_param("~P_0_yy", 0.0),
            rospy.get_param("~P_0_tt", 0.0),
        ])
        Q = np.diag([rospy.get_param("~Q_xx", 0.0), rospy.get_param("~Q_tt", 0.0)])
        R = np.diag([rospy.get_param("~R_rr", 0.0), rospy.get_param("~R_tt", 0.0)])
        print("RECIEVED INIT POS:", q_0)
        self.ekf = EKF(q_0, P_0, Q, R)

        tag_map_param = rospy.get_param("~map", None)
        print(tag_map_param)
        if tag_map_param is None:
            rospy.logerr("No AprilTag map provided (~map)")
            rospy.signal_shutdown("No AprilTag map provided")
        # print(os.getcwd())
        # print(os.listdir(os.getcwd()))
        # print(os.listdir('packages'))
        # with open("packages/navigation/src/map.yaml") as f:
        #     data = yaml.safe_load(f)
        #     tag_map_param = data['map']
        self.tag_map = {int(k): np.array(v["position"]) for k, v in tag_map_param.items()}

        self.apriltag_detector = Detector(
            families="tag36h11", nthreads=1, quad_decimate=2.0,
            quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25,
        )

        # --- navigation / graph state ---
        self.graph = load_graph_from_yaml(rospy.get_param("~map_file"))
        # self.graph = load_graph_from_yaml("packages/navigation/src/graphs/virtual.yaml")
        self.goal_xy = None
        # self.arrival_radius = rospy.get_param("~arrival_radius", 1.00)  # meters
        self.arrival_radius = 0.30
        self._awaiting_arrival = False

        # --- subscribers ---
        rospy.Subscriber(f"/{self.veh}/camera_node/image/compressed", CompressedImage,
                          self.cb_image, buff_size=10_000_000, queue_size=1)
        self.sub_camera_info = rospy.Subscriber(
            f"/{self.veh}/camera_node/camera_info", CameraInfo, self.cb_info, queue_size=1)
        rospy.Subscriber(f"/{self.veh}/left_wheel_encoder_driver_node/tick",
                          WheelEncoderStamped, self.cb_left_encoder)
        rospy.Subscriber(f"/{self.veh}/right_wheel_encoder_driver_node/tick",
                          WheelEncoderStamped, self.cb_right_encoder)
        rospy.Subscriber(f"/{self.veh}/duckiematrix_interface_node/state",
                          Odometry, self.cb_gt_pose, queue_size=1)
        rospy.Subscriber(f"/{self.veh}/navigation/goal", String, self.cb_goal)

        # --- publishers ---
        self.pub_pose = rospy.Publisher(
            f"/{self.veh}/ekf_localization_node/pose", Odometry, queue_size=1,
            dt_topic_type=TopicType.LOCALIZATION, latch=True)
        self.turn_queue_pub = rospy.Publisher(
            f"/{self.veh}/navigation/turn_queue", String, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(
            f"/{self.veh}/navigation/status", String, queue_size=1, latch=True)
        self.arrived_pub = rospy.Publisher(
            f"/{self.veh}/navigation/arrived", Bool, queue_size=1, latch=True)
        self.landmarks_pub = rospy.Publisher(
            f"/{self.veh}/navigation/landmarks", String, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher(
            f"/{self.veh}/navigation/path", String, queue_size=1, latch=True)
        self.detected_pub = rospy.Publisher(
            f"/{self.veh}/navigation/detected_landmarks", String, queue_size=1)

        rospy.sleep(0.5)
        self.publish_pose()
        self._publish_status("READY")

        rospy.Timer(rospy.Duration(1.0 / 10.0), self.do_predict)
        self.loginfo("Initialized!")

        landmarks = [{"id": tid, "x": float(pos[0]), "y": float(pos[1])}
                     for tid, pos in self.tag_map.items()]
        self.landmarks_pub.publish(String(data=json.dumps(landmarks)))

    # ---------------------------------------------------------------- #
    # Localization
    # ---------------------------------------------------------------- #
    def _reset_encoder_state(self):
        self.delta_phi_left = 0.0
        self.left_tick_prev = None
        self.delta_phi_right = 0.0
        self.right_tick_prev = None

    def cb_left_encoder(self, msg):
        with self.left_wheel_mutex:
            if self.left_tick_prev is None:
                self.left_tick_prev = msg.data
                return
            d = delta_phi(msg.data, self.left_tick_prev, msg.resolution)
            if d == 0:
                return
            self.left_tick_prev = msg.data
            self.delta_phi_left += d

    def cb_right_encoder(self, msg):
        with self.right_wheel_mutex:
            if self.right_tick_prev is None:
                self.right_tick_prev = msg.data
                return
            d = delta_phi(msg.data, self.right_tick_prev, msg.resolution)
            if d == 0:
                return
            self.right_tick_prev = msg.data
            self.delta_phi_right += d

    def cb_info(self, msg):
        self.loginfo("Camera info received, unsubscribing.")
        try:
            self.sub_camera_info.unregister()
        except BaseException:
            pass
        self.H, self.W = msg.height, msg.width
        self.K = np.reshape(msg.K, (3, 3))
        self.D = np.reshape(msg.D, (5,))
        self.P = np.reshape(msg.P, (3, 4))
        self.rect_camera_K, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, (self.W, self.H), 0.0)

    def cb_image(self, msg):
        self.latest_img = msg

    def cb_gt_pose(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        p = msg.pose.pose.position
        self.gt_pose = [p.x, p.y, yaw]

    def do_predict(self, event=None):
        if self.delta_phi_right == 0 and self.delta_phi_left == 0:
            return
        with self.left_wheel_mutex, self.right_wheel_mutex:
            dX, dT = get_odometry(self.R, self.baseline, self.delta_phi_left, self.delta_phi_right)
            self.delta_phi_left = 0.0
            self.delta_phi_right = 0.0
        with self.ekf_lock:
            self.ekf.predict(dX, dT)
        self.publish_pose()
        self.do_update()

    def do_update(self):
        if self.K is None or self.latest_img is None:
            return

        image_rgb = compressed_imgmsg_to_rgb(self.latest_img)
        image_gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]

        detections = self.apriltag_detector.detect(
            image_gray, estimate_tag_pose=True,
            camera_params=[fx, fy, cx, cy], tag_size=0.065)

        detected_ids = [det.tag_id for det in detections if det.tag_id in self.tag_map]
        self.detected_pub.publish(String(data=json.dumps(detected_ids)))

        with self.ekf_lock:
            for det in detections:
                if det.tag_id not in self.tag_map:
                    continue
                tag_x, tag_y = self.tag_map[det.tag_id][:2]
                if self.sim:
                    if self.gt_pose is None:
                        continue
                    dx, dy = tag_x - self.gt_pose[0], tag_y - self.gt_pose[1]
                    range_estimate = float(np.linalg.norm([dx, dy]))
                    bearing = wrap_angle(np.arctan2(dy, dx) - self.gt_pose[2])
                else:
                    t = det.pose_t
                    range_estimate = float(np.linalg.norm(t))
                    bearing = wrap_angle(-np.arctan2(t[0, 0], t[2, 0]))
                self.ekf.update([range_estimate, bearing], [tag_x, tag_y])

        self.publish_pose(self.latest_img.header)
        self._check_arrival()

    def publish_pose(self, header=None):
        with self.ekf_lock:
            q, P = self.ekf.q.copy(), self.ekf.P.copy()

        pose_cov = PoseWithCovariance()
        pose_cov.pose.position.x, pose_cov.pose.position.y = q[0], q[1]
        pose_cov.pose.orientation.z = np.sin(q[2] / 2)
        pose_cov.pose.orientation.w = np.cos(q[2] / 2)
        pose_cov.covariance = [
            P[0, 0], P[0, 1], 0.0, 0.0, 0.0, P[0, 2],
            P[1, 0], P[1, 1], 0.0, 0.0, 0.0, P[1, 2],
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            P[2, 0], P[2, 1], 0.0, 0.0, 0.0, P[2, 2],
        ]
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now() if header is None else header.stamp
        odom_msg.header.frame_id = "map"
        odom_msg.pose = pose_cov
        self.pub_pose.publish(odom_msg)

    # ---------------------------------------------------------------- #
    # Navigation - reads self.ekf.q directly, no subscription needed
    # ---------------------------------------------------------------- #
    def cb_goal(self, msg):
        try:
            x_str, y_str = msg.data.split(",")
            gx, gy = float(x_str), float(y_str)
        except Exception as e:
            rospy.logerr(f"[navigation] bad goal '{msg.data}': {e}")
            self._publish_status(f"GOAL_ERROR: {e}")
            return
        self._plan_and_publish(gx, gy)

    def _plan_and_publish(self, gx, gy):
        with self.ekf_lock:
            x, y, theta = self.ekf.q.copy()
            print("robot THETA", theta)

        self.graph.restore_graph(["START", "GOAL"])  # drop any earlier splice
        start = self.graph.add_node_with_splice("START", x, y)
        goal = self.graph.add_node_with_splice("GOAL", gx, gy)

        nodes_seq, edges_seq = self.graph.shortest_path(start, goal)
        turns = self.graph.path_to_turns(edges_seq, nodes_seq, Direction.from_angle(theta))

        self.goal_xy = (gx, gy)
        self._awaiting_arrival = True
        self.arrived_pub.publish(Bool(data=False))

        rospy.loginfo(f"[navigation] path {nodes_seq}, turns={turns}")
        self.turn_queue_pub.publish(String(data=",".join(turns)))
        path_world = [[self.graph.nodes[n].x, self.graph.nodes[n].y] for n in nodes_seq]
        self.path_pub.publish(String(json.dumps(path_world)))
        self._publish_status(f"EN_ROUTE to ({gx:.2f}, {gy:.2f}) via {nodes_seq}")

    def _check_arrival(self):
        if not self._awaiting_arrival or self.goal_xy is None:
            return
        with self.ekf_lock:
            x, y = self.ekf.q[0], self.ekf.q[1]
        gx, gy = self.goal_xy
        dist = math.hypot(x - gx, y - gy)
        if dist <= self.arrival_radius:
            print("ARRIVED")
            self._awaiting_arrival = False
            self.arrived_pub.publish(Bool(data=True))
            self._publish_status("ARRIVED")

    def _publish_status(self, text):
        self.status_pub.publish(String(data=text))


if __name__ == "__main__":
    node = NavigationNode(node_name="navigation_node")
    rospy.spin()