#encoding: utf-8
import numpy as np
import math
import matplotlib.pyplot as plt

############################# 需要标定的参数 ######################
angle = -180.3
extrin_x = -0.89
extrin_y = -0.16
angle_increment_lidar1 = 0.2495
angle_increment_lidar2 = 0.2495
#################################################################

lidar1_path = 'lidar_data/lidar1_data.txt'
lidar2_path = 'lidar_data/lidar2_data.txt'

lidar1_angle = []
lidar1_range = []
lidar2_angle = []
lidar2_range = []

lidar1_x = [0] * 1080
lidar1_y = [0] * 1080
lidar2_x = [0] * 1080
lidar2_y = [0] * 1080

with open(lidar1_path, 'r') as lidar1_data:
    lines = lidar1_data.readlines()
    for line in lines:
        value = [float(s) for s in line.split()]
        lidar1_angle.append(value[0])
        lidar1_range.append(value[1])

with open(lidar2_path, 'r') as lidar2_data:
    lines = lidar2_data.readlines()
    for line in lines:
        value = [float(s) for s in line.split()]
        lidar2_angle.append(value[0])
        lidar2_range.append(value[1])

for i in range(len(lidar1_angle)):
    if lidar1_range[i] < 20:
        lidar1_x[i] = lidar1_range[i] * math.cos(lidar1_angle[i] / 180 * math.pi)
        lidar1_y[i] = lidar1_range[i] * math.sin(lidar1_angle[i] / 180 * math.pi)
    if lidar2_range[i] < 20:
        lidar2_x[i] = lidar2_range[i] * math.cos(lidar2_angle[i] / 180 * math.pi)
        lidar2_y[i] = lidar2_range[i] * math.sin(lidar2_angle[i] / 180 * math.pi)

theta = math.radians(angle)
R = np.array([[math.cos(theta), -math.sin(theta)],[math.sin(theta), math.cos(theta)]])
pointcloud_lidar1 = np.array([lidar1_x , lidar1_y])
pointcloud_lidar2 = np.array([lidar2_x , lidar2_y])
extrin = np.tile(np.array([[extrin_x], [extrin_y]]), (1, len(lidar1_x)))
pointcloud_lidar2 = np.matmul(R , pointcloud_lidar2) + extrin


plt.plot(pointcloud_lidar1[0], pointcloud_lidar1[1], 'b.', label = "lidar1")
plt.plot(pointcloud_lidar2[0], pointcloud_lidar2[1], 'r.', label = "lidar2")
plt.title('Calibration')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.axis('equal')
plt.show()