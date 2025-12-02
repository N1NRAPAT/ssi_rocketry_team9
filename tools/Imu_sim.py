import serial
import serial.tools.list_ports
import csv
import time
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import radians
import os

# Folder to save CSV
SAVE_FOLDER = "/Users/punnaratsuttinual/SSI_rocketry/data/"

# Auto-detect Pico serial port
def find_pico_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if ("Pico" in p.description or
            "usbmodem" in p.device or
            "ttyACM" in p.device or
            "USB Serial Device" in p.description):
            return p.device
    return None

print(" Searching for Pico serial port…")
port = find_pico_port()

if port is None:
    print("ERROR: No Pico detected. Plug it in and try again.")
    exit()

print(f" Pico detected on: {port}")

# Open serial
ser = serial.Serial(port, 115200)

# IMU collection window 
START_MS = 30000 # start at 30s 
STOP_MS = 60000 # stop at 60s

# Create clean CSV name
timestamp = datetime.now().strftime("%H_%M_%S")
csv_path = os.path.join(SAVE_FOLDER, f"imu_flight_{timestamp}.csv")

csv_file = open(csv_path, "w", newline="")
writer = csv.writer(csv_file)
writer.writerow(["ax", "ay", "az", "gx", "gy", "gz", "time_ms"])

print("Logging + Simulation started!")

# Setup live plot
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# cube model generate from gpt 
cube = np.array([
    [-1, -1, -1],
    [-1, -1,  1],
    [-1,  1, -1],
    [-1,  1,  1],
    [ 1, -1, -1],
    [ 1, -1,  1],
    [ 1,  1, -1],
    [ 1,  1,  1],
])

# Rotate cube in real time 
def rotate(points, rx, ry, rz):
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)]
    ])
    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])
    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1]
    ])
    return points @ Rx @ Ry @ Rz


# MAIN LOOP

start_time = time.time() # Set timer 

while True:
    line = ser.readline().decode().strip() # Read via usb port from imu sensor
    parts = line.split(",")

    if len(parts) != 6: # ax, ay, az, gx, gy, gz
        continue

    ax_val, ay_val, az_val, gx, gy, gz = map(int, parts)
    now = int((time.time() - start_time) * 1000) #convert timmer to real time 


    # 1. WRITE CSV (only 30s–60s)
    if START_MS <= now <= STOP_MS:
        writer.writerow([ax_val, ay_val, az_val, gx, gy, gz, now])

    if now > STOP_MS:
        print("Finished logging!")
        break

    # 2. REAL-TIME 3D SIMULATION
    rx = radians(gx * 0.0007)
    ry = radians(gy * 0.0007)
    rz = radians(gz * 0.0007)
    rot = rotate(cube, rx, ry, rz)

    ax.clear()
    ax.scatter(rot[:, 0], rot[:, 1], rot[:, 2], c='black')
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])
    ax.set_title("Real-time IMU Orientation")

    plt.draw()
    plt.pause(0.001)

csv_file.close()
print(f"Saved CSV → {csv_path}")
