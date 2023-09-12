import os
import math
import numpy as np

folders_path = '/home/wen/workspace/predictions'

file_list = [i for i in os.listdir(folders_path) if i.endswith('.npy')]

target_position = (11,11)
x = target_position[0]
y = target_position[1]
target_h = 1.445
target_w = 1.879
target_l = 4.963
original_yaw_degrees = -90

for file_name in file_list:
    position_str = file_name[15:] #11.7_7.1
    robot_x = float(position_str[:4])
    robot_y = float(position_str[5:-4])

    x_position_robot = math.sqrt((x-robot_x)**2 + (y-robot_y)**2)
    y_position_robot = float(0)
    z_position_robot = float(0)

    theta = math.atan2(y-robot_y, x-robot_x)
    beta = np.pi - theta
    
    yaw = beta + np.pi/2

    corret_result = (x_position_robot, y_position_robot, z_position_robot, target_h, target_w, target_l, yaw)
    
