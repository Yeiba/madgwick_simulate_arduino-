import struct
import time
import numpy as np
from vpython import *
from simulate import simulate
from stream import stream
from MadgwickAHRS import MadgwickAHRS

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
    dataPacket = simulate.simulate_arduino_data()  # Get the simulated data
    # dataPacket = stream.stream_arduino_data()  # Get the stream data
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
