#!/usr/bin/env python3
"""
Unified EKF for the combined VO + AprilTag navigation node.

State: q = [x, y, theta]  (world frame)

Prediction (mutually exclusive per cycle):
  - predict_vo(v, delta_theta_vo, dt):  PRIMARY. Heading change from visual
        odometry (essential-matrix decomposition) + wheel-encoder linear
        velocity. Midpoint integration. Same math as the original
        VisualOdometry.EKF.predict_vo().
  - predict_wheel(dX, dT):              FALLBACK. Pure differential-drive
        wheel odometry — used whenever VO can't produce a usable heading
        estimate this cycle. Same math as the original
        NavigationNode.EKF.predict().

Correction:
  - update_apriltag(z, tag_xy):         Range/bearing correction from a
        detected AprilTag with known map position. Unchanged from the
        original NavigationNode.EKF.update().
"""

import numpy as np
from multiprocessing import Lock


def wrap_angle(a):
    """Wrap angle to (-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


class EKF:
    def __init__(self, q_0: np.ndarray, P_0: np.ndarray,
                 Q_wheel: np.ndarray, R_apriltag: np.ndarray,
                 Q_vo: np.ndarray = None):
        self.q = np.array(q_0, dtype=float)
        self.P = np.array(P_0, dtype=float)

        # Process noise for the wheel-only fallback predict (2x2, on the
        # [dX, dT] inputs, mapped into state space via W below) — this is
        # exactly the Q that used to live in NavigationNode.EKF.
        self.Q_wheel = Q_wheel
        self.Q_wheel_sqrt = np.sqrt(Q_wheel)

        # Process noise for VO-driven predict, expressed directly in state
        # space [x, y, theta] since VO already gives us a heading *change*
        # rather than raw wheel inputs. Defaults match the original
        # VisualOdometry.EKF.Q_vo.
        self.Q_vo = Q_vo if Q_vo is not None else np.diag([0.02, 0.02, 0.2]) ** 2

        # Measurement noise for AprilTag range/bearing updates.
        self.R = R_apriltag

        # Single lock serializes all access — the node no longer needs its
        # own separate ekf_lock (that was redundant double-locking).
        self.q_mutex = Lock()

        self._theta_before_predict = self.q[2]

    # ------------------------------------------------------------------ #
    # Prediction
    # ------------------------------------------------------------------ #
    def predict_wheel(self, dX, dT):
        """Fallback predict from differential-drive wheel odometry."""
        with self.q_mutex:
            theta = self.q[2]
            self._theta_before_predict = theta

            self.q[0] += dX * np.cos(theta)
            self.q[1] += dX * np.sin(theta)
            self.q[2] = wrap_angle(theta + dT)

            F = np.array([
                [1, 0, -dX * np.sin(theta)],
                [0, 1,  dX * np.cos(theta)],
                [0, 0, 1],
            ])
            W = np.array([
                [np.cos(theta), 0],
                [np.sin(theta), 0],
                [0, 1],
            ])
            
            Q_dyanimc = self.Q_wheel_sqrt * np.array([[dX, 0],
                                                      [0, dT]])

            Q_dyanimc = Q_dyanimc * Q_dyanimc

            self.P = F @ self.P @ F.T + W @ Q_dyanimc @ W.T
            self._wrap_theta_locked()

    def predict_vo(self, v, delta_theta_vo, dt):
        """Primary predict driven by VO heading change + wheel-encoder
        velocity. Midpoint (trapezoidal) integration."""
        with self.q_mutex:
            theta = self.q[2]
            self._theta_before_predict = theta

            theta_mid = wrap_angle(theta + delta_theta_vo / 2.0)
            theta_new = wrap_angle(theta + delta_theta_vo)

            self.q[0] += v * np.cos(theta_mid) * dt
            self.q[1] += v * np.sin(theta_mid) * dt
            self.q[2] = theta_new

            F = np.array([
                [1, 0, -v * np.sin(theta_mid) * dt],
                [0, 1,  v * np.cos(theta_mid) * dt],
                [0, 0, 1],
            ])
            self.P = F @ self.P @ F.T + self.Q_vo * dt
            self._wrap_theta_locked()

    # ------------------------------------------------------------------ #
    # Correction
    # ------------------------------------------------------------------ #
    def update_apriltag(self, z: np.ndarray, tag_xy):
        """Range/bearing correction from a detected AprilTag."""
        with self.q_mutex:
            dx = tag_xy[0] - self.q[0]
            dy = tag_xy[1] - self.q[1]
            rng_pred = np.sqrt(dx * dx + dy * dy)
            bearing_pred = wrap_angle(np.arctan2(dy, dx) - self.q[2])
            z_pred = np.array([rng_pred, bearing_pred])

            y = z - z_pred
            y[1] = wrap_angle(y[1])

            r2 = rng_pred * rng_pred
            H = np.array([
                [-dx / rng_pred, -dy / rng_pred, 0],
                [dy / r2, -dx / r2, -1],
            ])

            S = H @ self.P @ H.T + self.R
            K = self.P @ H.T @ np.linalg.inv(S)

            self.q = self.q + K @ y
            self.q[2] = wrap_angle(self.q[2])

            I_KH = np.eye(self.q.shape[0]) - K @ H
            self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

    # ------------------------------------------------------------------ #
    def _wrap_theta_locked(self):
        """Caller must already hold q_mutex."""
        self.q[2] = wrap_angle(self.q[2])

    def get_state(self):
        with self.q_mutex:
            return self.q.copy()