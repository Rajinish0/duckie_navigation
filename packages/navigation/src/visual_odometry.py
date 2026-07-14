#!/usr/bin/env python3
import os
import cv2
import numpy as np


class VisualOdometry:
    """
    ROS-free implementation of Visual Odometry processing.

    Predicts into an externally-owned, shared EKF (see ekf.py) rather than
    keeping its own — this lets AprilTag corrections (handled elsewhere in
    the node) operate on the same fused state.
    """

    # Maximum plausible heading change per frame (rad/s worth), used to
    # gate obviously-bad VO rotation estimates before they hit the filter.
    MAX_HEADING_RATE = 20.0

    def __init__(self, intrinsic_matrix, ekf, output_dir=None, save_matches=True,
                 mask_top_ratio=None):
        self.K = intrinsic_matrix
        self.orb = cv2.ORB_create()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        self.prev_image = None
        self.prev_keypoints = None
        self.prev_descriptors = None

        # Shared EKF instance, injected.
        self.ekf = ekf

        self.R_total = np.eye(3)
        self.t_total = np.zeros((3, 1))
        self.scale = 1.0

        self.R_latest = None
        self.t_latest = None
        self.latest_match_img = None

        self.output_dir = output_dir
        self.save_matches = save_matches
        self.mask_top_ratio = mask_top_ratio
        if self.output_dir:
            os.makedirs(self.output_dir, exist_ok=True)

        self.trajectory = []  # (x, y, theta)
        self.camera_tilt_rad = np.pi / 12

    def _get_kp_desc(self, image):
        return self.orb.detectAndCompute(image, None)

    def process_frame(self, image, v=0.0, dX=0.0, dT_wheel=0.0, dt=1.0,
                       frame_idx=0, scale_override=None, predict=True) -> (bool, str):
        """
        Process one frame.

        Predict strategy against the shared EKF:
          1. Attempt VO: compute delta_theta from the essential-matrix rotation.
          2. If VO succeeds  -> ekf.predict_vo(v, delta_theta_vo, dt)
          3. If VO fails      -> ekf.predict_wheel(dX, dT_wheel)  [wheel
                                  fallback, so the filter never stalls
                                  between frames even without VO]

        Args:
            v:         linear velocity from wheel encoders (m/s), used by VO.
            dX:        wheel-odometry translation since last frame (m),
                       used only for the fallback predict.
            dT_wheel:  wheel-odometry heading change since last frame (rad),
                       used only for the fallback predict.
        """
        self.latest_match_img = None

        if len(image.shape) == 3:
            image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            image_gray = image

        if self.mask_top_ratio is not None and self.mask_top_ratio > 0:
            mask_half_image = np.zeros(image_gray.shape, dtype=np.uint8)
            mask_half_image[
                    int(image_gray.shape[0] * self.mask_top_ratio):, :] = 255
            image_gray = cv2.bitwise_and(image_gray, image_gray,
                                         mask=mask_half_image)

        if self.prev_image is None:
            self.prev_image = image_gray
            self.prev_keypoints, self.prev_descriptors = self._get_kp_desc(image_gray)
            self.trajectory.append(tuple(self.ekf.get_state()))
            return False, "First frame: no VO possible"
        _diff = np.sum(np.abs(self.prev_image - image_gray))
        if _diff < 1.2:
            return False, f"Not much difference in image = {_diff:.4f}"

        keypoints, descriptors = self._get_kp_desc(image_gray)
        matches = []
        if self.prev_descriptors is not None and descriptors is not None:
            matches = self.bf.match(self.prev_descriptors, descriptors)
            matches = sorted(matches, key=lambda m: m.distance)
            matches = [m for m in matches if m.distance < 50]

        if len(matches) < 8:
            if predict:
                self.ekf.predict_wheel(dX, dT_wheel)
            self.prev_image, self.prev_keypoints, self.prev_descriptors = image_gray, keypoints, descriptors
            self.trajectory.append(tuple(self.ekf.get_state()))
            return False, f"Frame {frame_idx}: Not enough matches ({len(matches)}) for VO"

        pts_prev = np.float32([self.prev_keypoints[m.queryIdx].pt for m in matches])
        pts_curr = np.float32([keypoints[m.trainIdx].pt for m in matches])

        E, mask = cv2.findEssentialMat(
            pts_prev, pts_curr, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0
        )

        if E is not None:
            mask = mask.ravel().astype(bool)
            pts_prev_inliers = pts_prev[mask]
            pts_curr_inliers = pts_curr[mask]

            if pts_prev_inliers.shape[0] < 8:
                if predict:
                    self.ekf.predict_wheel(dX, dT_wheel)
                self.prev_image, self.prev_keypoints, self.prev_descriptors = image_gray, keypoints, descriptors
                self.trajectory.append(tuple(self.ekf.get_state()))
                return False, f"Frame {frame_idx}: Not enough inliers ({pts_prev_inliers.shape[0]}) after essential matrix estimation"

            _, R, t, mask_pose = cv2.recoverPose(
                E, pts_prev_inliers, pts_curr_inliers, self.K
            )

            mask_pose = mask_pose.ravel().astype(bool)
            pts_prev_final = pts_prev_inliers[mask_pose]

            if pts_prev_final.shape[0] < 8:
                if predict:
                    self.ekf.predict_wheel(dX, dT_wheel)
                self.prev_image, self.prev_keypoints, self.prev_descriptors = image_gray, keypoints, descriptors
                self.trajectory.append(tuple(self.ekf.get_state()))
                return False, f"Frame {frame_idx}: Not enough inliers ({pts_prev_final.shape[0]}) after recoverPose"

            rvec, _ = cv2.Rodrigues(R)

            ct = np.cos(self.camera_tilt_rad)
            st = np.sin(self.camera_tilt_rad)
            Rx_inv = np.array([[1,  0,   0],
                               [0,  ct,  st],
                               [0, -st,  ct]])
            R_level = Rx_inv @ R @ Rx_inv.T

            delta_theta_vo = float(np.arctan2(-R_level[2, 0], R_level[0, 0]))

            max_delta = self.MAX_HEADING_RATE * dt if dt > 0 else 0.1
            vo_heading_valid = (abs(delta_theta_vo) <= max_delta)

            if predict:
                if vo_heading_valid:
                    self.ekf.predict_vo(v, delta_theta_vo, dt)
                else:
                    # VO heading change too large — treat as outlier, fall
                    # back to wheel odometry for this cycle.
                    self.ekf.predict_wheel(dX, dT_wheel)

            frame_scale = scale_override if scale_override is not None else self.scale
            self.t_total = self.t_total + frame_scale * (self.R_total @ t)
            self.R_total = self.R_total @ R
            self.R_latest = R
            self.t_latest = t

            if self.save_matches:
                self.latest_match_img = cv2.drawMatches(
                    self.prev_image, self.prev_keypoints,
                    image_gray, keypoints,
                    matches[:50], None,
                    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
                )
                state = self.ekf.get_state()
                cv2.putText(self.latest_match_img, f"Position: ({state[0]:.2f}, {state[1]:.2f})",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
                cv2.putText(self.latest_match_img, f"Heading : {np.degrees(state[2]):.2f} deg",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
                center = (self.latest_match_img.shape[1] // 4, 3 * self.latest_match_img.shape[0] // 4)
                heading_length = 50
                heading_angle = state[2]
                heading_end = (int(center[0] - heading_length * np.sin(heading_angle)),
                               int(center[1] - heading_length * np.cos(heading_angle)))
                cv2.arrowedLine(self.latest_match_img, center, heading_end, (0, 0, 255), 2, tipLength=0.2)

                if self.output_dir:
                    out_path = os.path.join(self.output_dir, f"matches_{frame_idx:04d}.png")
                    cv2.imwrite(out_path, self.latest_match_img)

            self.prev_image, self.prev_keypoints, self.prev_descriptors = image_gray, keypoints, descriptors
            self.trajectory.append(tuple(self.ekf.get_state()))
            return True, "Success"
        else:
            if predict:
                self.ekf.predict_wheel(dX, dT_wheel)
            self.prev_image, self.prev_keypoints, self.prev_descriptors = image_gray, keypoints, descriptors
            self.trajectory.append(tuple(self.ekf.get_state()))
            return False, f"Frame {frame_idx}: Essential matrix estimation failed"

    def get_relative_pose(self):
        if len(self.trajectory) < 2:
            return None
        x0, y0, th0 = self.trajectory[-2]
        x1, y1, th1 = self.trajectory[-1]
        dx_world = x1 - x0
        dy_world = y1 - y0
        dtheta = np.arctan2(np.sin(th1 - th0), np.cos(th1 - th0))
        cos_th0, sin_th0 = np.cos(th0), np.sin(th0)
        ds_forward = cos_th0 * dx_world + sin_th0 * dy_world
        ds_lateral = -sin_th0 * dx_world + cos_th0 * dy_world
        return (dx_world, dy_world, dtheta), (ds_forward, ds_lateral, dtheta)

    def get_trajectory(self):
        return np.array(self.trajectory)

    def render_trajectory_cv2(self, canvas_size=500, margin=20):
        traj = self.get_trajectory()
        if len(traj) < 2:
            return None
        xs, ys = traj[:, 0], traj[:, 1]
        x_min, x_max = xs.min(), xs.max()
        y_min, y_max = ys.min(), ys.max()
        x_range = max(x_max - x_min, 1e-3)
        y_range = max(y_max - y_min, 1e-3)
        scale = (canvas_size - 2 * margin) / max(x_range, y_range)

        def to_px(x, y):
            px = int(margin + (x - x_min) * scale)
            py = int(canvas_size - margin - (y - y_min) * scale)
            return (px, py)

        img = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8)
        for i in range(0, canvas_size, canvas_size // 4):
            cv2.line(img, (i, 0), (i, canvas_size), (40, 40, 40), 1)
            cv2.line(img, (0, i), (canvas_size, i), (40, 40, 40), 1)

        pts = [to_px(x, y) for x, y in zip(xs, ys)]
        for i in range(1, len(pts)):
            t = i / max(len(pts) - 1, 1)
            colour = (int(255 * (1 - t)), int(255 * t), 0)
            cv2.line(img, pts[i - 1], pts[i], colour, 2)

        step = max(1, len(traj) // 10)
        for i in range(0, len(traj), step):
            x, y, theta = traj[i]
            px, py = to_px(x, y)
            arrow_len = int(0.05 * scale)
            ex = px + int(arrow_len * np.cos(theta))
            ey = py - int(arrow_len * np.sin(theta))
            cv2.arrowedLine(img, (px, py), (ex, ey), (0, 0, 255), 1, tipLength=0.4)

        cv2.circle(img, pts[0],  5, (255, 255, 0), -1)
        cv2.circle(img, pts[-1], 5, (0,   255, 255), -1)

        state = self.ekf.get_state()
        cv2.putText(img, f"Pos: ({state[0]:.2f}, {state[1]:.2f}) m",
                    (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        cv2.putText(img, f"Hdg: {np.degrees(state[2]):.1f} deg",
                    (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        cv2.putText(img, f"Frames: {len(traj)}",
                    (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        return img