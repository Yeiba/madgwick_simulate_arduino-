import struct
import serial


class stream:
    def stream_arduino_data():
        # Open serial port
        # Adjust COM port and baud rate as necessary
        arduinoData = serial.Serial('COM3', 9600, timeout=1)

        # Read a line from the serial port
        while True:
            if arduinoData.in_waiting > 0:
                dataPacket = arduinoData.readline().decode('utf-8').strip()

                # Split the packet into individual values
                splitPacket = dataPacket.split(',')

                # Ensure that the data has the correct number of elements
                if len(splitPacket) >= 6:
                    try:
                        # Convert strings to floats
                        ax, ay, az = map(float, splitPacket[:3])
                        gx, gy, gz = map(float, splitPacket[3:6])
                        # Set default values for magnetometer data if not available
                        mx, my, mz = 0.0, 0.0, 0.0

                        # Pack the data into a structure
                        data = struct.pack('9f', ax, ay, az, gx,
                                           gy, gz, mx, my, mz)
                        return data

                    except ValueError:
                        print("Error: Invalid data format")
                else:
                    print("Unexpected data format:", dataPacket)
