import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import radians

# change to your Pico port
ser = serial.Serial("/dev/tty.usbmodem0000000000001", 115200)

# cube points
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

# setup plot
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

while True:
    line = ser.readline().decode().strip()
    ax_val, ay_val, az_val, gx, gy, gz = map(int, line.split(","))

    # convert gyro to small rotation angles
    rx = radians(gx * 0.0005)
    ry = radians(gy * 0.0005)
    rz = radians(gz * 0.0005)

    rot = rotate(cube, rx, ry, rz)

    ax.clear()
    ax.scatter(rot[:,0], rot[:,1], rot[:,2], c='black')

    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])

    plt.draw()
    plt.pause(0.01)
