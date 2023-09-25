import os
import math
import numpy as np

target_position = (11,11)
x = target_position[0]
y = target_position[1]
target_h = 1.445
target_w = 1.879
target_l = 4.963

folders_path = '/home/wen/catkin_ws/src/p3at_gazebo/data_space/gazebo_lidar/09251722'

input_path =os.path.join(folders_path, 'robot_coordiantes.txt') 
output_path = os.path.join(folders_path, 'label.txt')
with open(input_path, 'r') as file:
    with open(output_path, 'w') as output_file:
    
        # 逐行读取文件内容
        for line in file:
            parts = line.strip().split()
            # 解析数据
            file_name, robot_x, robot_y=str(parts[2]), float(parts[0]), float(parts[1]) # 世界坐标系
            
            # 机器人坐标系
            x_position_robot = math.sqrt((x-robot_x)**2 + (y-robot_y)**2)
            y_position_robot = float(0)
            z_position_robot = float(0)

            theta = math.atan2(y-robot_y, x-robot_x)
            beta = np.pi - theta
            
            yaw = beta + np.pi/2

            # 正确的结果写入txt
            output_file.write(f"{file_name} {x_position_robot} {y_position_robot} {z_position_robot} {target_h} {target_w} {target_l} {yaw}\n")

print("Done")
       