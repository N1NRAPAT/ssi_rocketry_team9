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

# ----------- AUTO-DETECT PICO SERIAL PORT (FINAL FIXED VERSION) -----------
def find_pico_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = p.description.lower()
        dev = p.device.lower()

        # Check descriptions OR typical Mac serial names
        if ("pico" in desc or
            "rp2" in desc or
            "usb serial" in desc or
            "usbmodem" in dev):

            # Prefer /tty.* but accept /cu.* fallback
            if "/tty." in dev:
                return p.device
            if "/cu." in dev:
                return p.device

    return None
# ---------------------------------------------------------------------------

print(" Searching for Pico serial port…")

port = None
while port is None:
    port = find_pico_port()
    if port is None:
        print(" Waiting for Pico...")
        time.sleep(0.5)

print(f"Pico detected on: {port}")

# Open serial port
ser = serial.Serial(port, 115200)

# IMU logging window
START_MS = 30000
STOP_MS = 60000

# CSV setup + file name
timestamp = datetime.now().strftime("%H_%M_%S")
csv_path = os.path.join(SAVE_FOLDER, f"imu_flight_{timestamp}.csv")
csv_file = open(csv_path, "w", newline="")
writer = csv.writer(csv_file)
writer.writerow(["ax", "ay", "az", "gx", "gy", "gz", "time_ms"])

print(" Logging + Simulation started!")

# Plot setup
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

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
start_time = time.time()

while True:
    line = ser.readline().decode().strip()
    parts = line.split(",")

    if len(parts) != 6:
        continue

    ax_val, ay_val, az_val, gx, gy, gz = map(int, parts)
    now = int((time.time() - start_time) * 1000)

    # CSV logging (30–60s)
    if START_MS <= now <= STOP_MS:
        writer.writerow([ax_val, ay_val, az_val, gx, gy, gz, now])

    if now > STOP_MS:
        print(" Finished logging!")
        break

    # Real-time cube rotation
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
print(f" Saved CSV → {csv_path}")
