#!/usr/bin/env python

#x,y

import os
import sys
import time
import math

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

rospy.init_node('get_points_once')

# 目标位置
target_position = (11, 11)

#距离目标的半径
step = 0.5
radius_array = np.arange(4,7,step)

coordinate_robot = []
for radius in radius_array:
    for angle_degree in range(0,181,5):
        angle_radians = math.radians(angle_degree)
        
        # 使用三角函数计算点的坐标
        x = target_position[0] + radius * math.sin(angle_radians)
        y = target_position[1] + radius * math.cos(angle_radians)
        # 使用round函数保留一位小数
        x = round(x, 1)
        y = round(y, 1)
        coordinate_robot.append([x,y])
print(f'len(coordinate_robot):{len(coordinate_robot)}')


for idx, [x,y] in enumerate(coordinate_robot):
    w = - (y-target_position[0]) / np.sqrt((y-target_position[1])**2 + (x-target_position[0])**2)

    # 当前位置
    current_position = (x, y)

    # 计算方向向量
    direction_vector = (target_position[0] - current_position[0], target_position[1] - current_position[1])

    # 计算旋转角度（以弧度为单位）
    rotation_angle = math.atan2(direction_vector[1], direction_vector[0])

    # 计算旋转四元数
    half_angle = rotation_angle / 2
    cos_half_angle = math.cos(half_angle)
    sin_half_angle = math.sin(half_angle)
    z = sin_half_angle
    w = cos_half_angle
    #robot_move
    cmd = "rosservice call /gazebo/set_model_state '{model_state: { model_name: example, pose: { position: { x: "+ str(x) +" , y: "+ str(y) +" , z: 0. }, orientation: {x: 0, y: 0., z: "+ str(z) +", w: "+ str(w) +" } }, twist: { linear: {x: 0.0, y: 0.0, z: 0.0 } , angular: { x: 0.0, y: 0.0, z: 0.0 } } , reference_frame: world } }'"
    os.system(cmd)
    # cmd = "clear"
    # os.system(cmd)
    print('{0}/{1}'.format(idx+1,len(coordinate_robot)))
    print('The pose of robot:{}_{} '.format(x,y))

    #   wait 0.5s
    time.sleep(0.5)


    #get_point
    msg = rospy.wait_for_message("/velodyne_points", PointCloud2, timeout=None)

    ##  Get pointcloud below this line    
    point_cloud = pc2.read_points(msg, field_names=(
        "x", "y", "z", "intensity"))#, skip_nans=False


    # save_point
    savepath = '/home/wen/workspace/gazebo_lidar_data'
    filename = 'point_'+str(x)+'_'+str(y)
    np.save(os.path.join(savepath,filename),np.array(list(point_cloud), dtype=np.float32))
    print("{} saved".format(filename))

    #   wait 1s
    time.sleep(1)

    if rospy.is_shutdown():
                sys.exit("killed")
