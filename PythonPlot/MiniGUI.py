import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import serial
import re

# Function to correct the decimal format
def correct_decimal(value_str):
    corrected = value_str.replace('.-', '.')
    return float(corrected)

# Set up the serial connection
ser = serial.Serial('COM3', 115200, timeout=1)

# Set up the plot for interactive mode
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Initial plot setup
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

while True:
    try:
        line = ser.readline().decode('utf-8').strip()
        # Parse the line using a regular expression to extract the orientation components
        match = re.search(r"Orient_X: ([\d.-]+) rad, Orient_Y: ([\d.-]+) rad, Orient_Z: ([\d.-]+) rad", line)
        if match:
            # Correct the decimal format and convert to float
            orient_x = correct_decimal(match.group(1))
            orient_y = correct_decimal(match.group(2))
            orient_z = correct_decimal(match.group(3))
            # Clear the plot for the new vector
            ax.cla()
            # Plot the new vector from origin to (orient_x, orient_y, orient_z)
            ax.quiver(0, 0, 0, orient_x, orient_y, orient_z, length=1, color='b', normalize=True)
            # Annotate the vector with its values
            ax.text(orient_x, orient_y, orient_z, f"({orient_x:.2f}, {orient_y:.2f}, {orient_z:.2f})", color='blue')
            # Reset the plot limits
            ax.set_xlim([-1, 1])
            ax.set_ylim([-1, 1])
            ax.set_zlim([-1, 1])
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            plt.draw()
            plt.pause(0.001)
    except KeyboardInterrupt:
        break
    except Exception as e:
        print(f"Error: {e}")

ser.close()
plt.ioff()
plt.show()

# C:\Users\vmyhr\AppData\Local\Programs\Python\Python312\python.exe MiniGui.py
