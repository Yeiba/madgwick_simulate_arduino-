import struct
import random


class simulate:
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
