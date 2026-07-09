#!/usr/bin/env python3

import numpy as np
from multiprocessing import Lock


def wrap_angle(a):
    """Wrap angle to [-pi, pi]."""
    return (a + np.pi) % (2*np.pi) - np.pi

class EKF:
    def __init__(self, q_0: np.ndarray, P_0: np.ndarray, Q: np.ndarray, R: np.ndarray):
        self.q = q_0
        self.P = P_0
        self.Q = Q
        self.R = R
        self.q_mutex = Lock()

    def predict(self, dX, dT):

        with self.q_mutex:

            theta = self.q[2]

            # State prediction
            self.q[0] += dX * np.cos(theta)
            self.q[1] += dX * np.sin(theta)
            self.q[2] = wrap_angle(theta + dT)

            # Jacobian wrt state
            F = np.array([
                [1, 0, -dX * np.sin(theta)],
                [0, 1,  dX * np.cos(theta)],
                [0, 0, 1]
            ])

            # Jacobian wrt process noise
            W = np.array([
                [np.cos(theta), 0],
                [np.sin(theta), 0],
                [0, 1]
            ])

            self.P = F @ self.P @ F.T + W @ self.Q @ W.T

    def update(self, z: np.ndarray, tag_xy: np.ndarray):
        # z is the measurement in the form [range, bearing]
        # tag_xy is the tag location of the tag in world coordinates [tag_x, tag_y]

        with self.q_mutex:

            # Step 1: calculate the predicted range and bearing measurements
            # TODO: update the equations below
            dx = tag_xy[0] - self.q[0]
            dy = tag_xy[1] - self.q[1]
            rng_pred = np.sqrt(dx * dx + dy * dy)
            bearing_pred = wrap_angle(np.arctan2(dy, dx) - self.q[2])
            z_pred = np.array([rng_pred, bearing_pred])

            # Step 2: Calculate the innovation
            # TODO: Define y
            y = z - z_pred
            y[1] = wrap_angle(y[1])

            # Step 3: Calculate the measurement Jacobian
            # TODO: Define H
            r2 = rng_pred * rng_pred
            H = np.array([
                [-dx / rng_pred, -dy / rng_pred, 0],
                [dy / r2, -dx / r2, -1]
            ])

            # Step 4: Calculate the Kalman gain
            # TODO: Define K
            S = H @ self.P @ H.T + self.R
            K = self.P @ H.T @ np.linalg.inv(S)

            # Step 5: Update the state and covariance estimates
            # TODO: Update these equations
            self.q = self.q + K @ y
            self.q[2] = wrap_angle(self.q[2])

            I_KH = np.eye(self.q.shape[0]) - K@H
            self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T






