import struct
import numpy as np


# Madgwick filter implementation


class MadgwickAHRS:
    def __init__(self, sampleperiod=1/100, beta=0.1):
        self.beta = beta
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.sampleperiod = sampleperiod

    def update_imu(self, gyro, accel):
        q = self.q
        sampleperiod = self.sampleperiod
        beta = self.beta

        gx, gy, gz = np.radians(gyro)
        ax, ay, az = accel

        norm = np.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0:
            return
        ax, ay, az = ax / norm, ay / norm, az / norm

        q1, q2, q3, q4 = q

        # Auxiliary variables
        _2q1q3 = 2.0 * (q1 * q3 - q2 * q4)
        _2q2q4 = 2.0 * (q2 * q4 - q1 * q3)
        _4q1q2 = 4.0 * (q1 * q2)
        _4q2q3 = 4.0 * (q2 * q3)
        _8q2q4 = 8.0 * (q2 * q4)
        _2q1q4 = 2.0 * (q1 * q4 + q2 * q3)
        _2q3q4 = 2.0 * (q3 * q4 - q1 * q2)
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        q4q4 = q4 * q4

        # Gradient descent algorithm corrective step
        f1 = 2.0 * (q2 * q4 - q1 * q3) - ax
        f2 = 2.0 * (q1 * q2 + q3 * q4) - ay
        f3 = 1.0 - 2.0 * (q2 * q2 + q3 * q3) - az

        J = np.array([
            [-2 * q4, 2 * q3, -2 * q2, 2 * q1],
            [2 * q1, 2 * q4, 2 * q3, 2 * q2],
            [0, -4 * q1, 4 * q2, 0],
            [4 * q3, 0, -4 * q1, 0]
        ])

        # Normalize the Jacobian matrix
        norm_J = np.linalg.norm(J, axis=1)
        norm_J[norm_J == 0] = 1  # Prevent division by zero
        J = J / norm_J[:, None]

        # Corrective step
        step = np.dot(J.T, np.array([f1, f2, f3, 0]))
        step = step / np.linalg.norm(step)

        q_dot = 0.5 * np.array([
            -q2 * gx - q3 * gy - q4 * gz,
            q1 * gx + q3 * gz - q4 * gy,
            q1 * gy - q2 * gz + q4 * gx,
            q1 * gz + q2 * gy - q3 * gx
        ])

        q_dot -= beta * step
        q_dot = q_dot / np.linalg.norm(q_dot)

        q += q_dot * sampleperiod
        q = q / np.linalg.norm(q)

        self.q = q

    @property
    def quaternion(self):
        return self.q
