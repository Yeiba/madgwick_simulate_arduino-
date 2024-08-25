import struct
import time
import numpy as np
from vpython import *
import random


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


# Initialize Madgwick filter
madgwick = MadgwickAHRS(sampleperiod=1/100, beta=0.1)

# Initialize the 3D scene
scene = canvas(background=color.gray(0.8))
scene.range = 5
scene.forward = vector(-1, -1, -1)
scene.width = 1080
scene.height = 720

# Create arrows to represent axes
xarrow = arrow(pos=vector(0, 0, 0), length=2, shaftwidth=0.1,
               color=color.red, axis=vector(1, 0, 0))
yarrow = arrow(pos=vector(0, 0, 0), length=2, shaftwidth=0.1,
               color=color.green, axis=vector(0, 1, 0))
zarrow = arrow(pos=vector(0, 0, 0), length=2, shaftwidth=0.1,
               color=color.blue, axis=vector(0, 0, 1))

# Initialize variables for displacement
x_disp, y_disp, z_disp = 0, 0, 0
x_vel, y_vel, z_vel = 0, 0, 0

# Gravity constant to normalize accelerometer readings
NorAxis = 1 / 9.8

# Function to simulate data (replacing serial data from Arduino)


def simulate_arduino_data():
    # Simulate accelerometer (ax, ay, az), gyroscope (gx, gy, gz), and magnetometer (mx, my, mz) readings
    ax = round(random.uniform(-2.0, 2.0), 2)
    ay = round(random.uniform(-2.0, 2.0), 2)
    az = round(random.uniform(-2.0, 2.0), 2)
    gx = round(random.uniform(-250.0, 250.0), 2)
    gy = round(random.uniform(-250.0, 250.0), 2)
    gz = round(random.uniform(-250.0, 250.0), 2)
    mx = round(random.uniform(-50.0, 50.0), 2)
    my = round(random.uniform(-50.0, 50.0), 2)
    mz = round(random.uniform(-50.0, 50.0), 2)

    # Pack the data as 9 floats, 4 bytes each, similar to the struct.pack method from Arduino
    data = struct.pack('9f', ax, ay, az, gx, gy, gz, mx, my, mz)
    return data

# Function to process incoming data


def process_data(data):
    global x_disp, y_disp, z_disp, x_vel, y_vel, z_vel
    # Parse the data
    ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack('9f', data)

    # Normalize accelerometer data
    ax, ay, az = ax * NorAxis, ay * NorAxis, az * NorAxis

    # Apply the Madgwick filter to get orientation
    madgwick.update_imu([gx, gy, gz], [ax, ay, az])
    quaternion = madgwick.quaternion

    # Calculate rotation matrix from quaternion
    rotation_matrix = np.array([
        [1 - 2*(quaternion[1]**2 + quaternion[2]**2), 2*(quaternion[0]*quaternion[1] - quaternion[2]
                                                         * quaternion[3]), 2*(quaternion[0]*quaternion[2] + quaternion[1]*quaternion[3])],
        [2*(quaternion[0]*quaternion[1] + quaternion[2]*quaternion[3]), 1 - 2*(quaternion[0] **
                                                                               2 + quaternion[2]**2), 2*(quaternion[1]*quaternion[2] - quaternion[0]*quaternion[3])],
        [2*(quaternion[0]*quaternion[2] - quaternion[1]*quaternion[3]), 2*(quaternion[1] *
                                                                           quaternion[2] + quaternion[0]*quaternion[3]), 1 - 2*(quaternion[0]**2 + quaternion[1]**2)]
    ])

    # Calculate displacement
    dt = 1 / 100  # Assuming 100 Hz sampling rate
    x_vel += ax * dt
    y_vel += ay * dt
    z_vel += az * dt

    x_disp += x_vel * dt
    y_disp += y_vel * dt
    z_disp += z_vel * dt

    return rotation_matrix


# Animation loop
while True:
    # Simulate getting data from Arduino
    dataPacket = simulate_arduino_data()  # Get the simulated data
    rotation_matrix = process_data(dataPacket)

    # Update the axes based on the rotation matrix
    xarrow.axis = vector(*rotation_matrix[:, 0])
    yarrow.axis = vector(*rotation_matrix[:, 1])
    zarrow.axis = vector(*rotation_matrix[:, 2])

    # Update the position based on the displacement
    pos_vector = vector(x_disp, y_disp, z_disp)
    xarrow.pos = pos_vector
    yarrow.pos = pos_vector
    zarrow.pos = pos_vector

    time.sleep(0.01)  # Control the loop rate
