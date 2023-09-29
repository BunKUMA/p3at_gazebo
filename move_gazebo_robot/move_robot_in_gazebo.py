#!/usr/bin/env python

import os
import sys
import time
import math
import time
import ros_numpy 

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2 ,Image
from PIL import Image as PILImage
from PIL import ImageEnhance

            
def moveRobot(radius, angle_radians):
    """通过命令行的方式,面朝车生成机器人

    Args:
        radius (_type_): 离车距
        angle_radians (_type_): 与车正朝向的夹角

    Returns:
        x, y: 机器人当前坐标
    """
    # 使用三角函数计算点的坐标
    x = target_position[0] + radius * math.sin(angle_radians)
    y = target_position[1] + radius * math.cos(angle_radians)
    x = round(x, 3)
    y = round(y, 3)

    # for idx, [x,y] in enumerate(coordinate_robot):
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
    # robot_move
    cmd = "rosservice call /gazebo/set_model_state '{model_state: { model_name: robot, pose: { position: { x: "+ str(x) +" , y: "+ str(y) +" , z: 0. }, orientation: {x: 0, y: 0., z: "+ str(z) +", w: "+ str(w) +" } }, twist: { linear: {x: 0.0, y: 0.0, z: 0.0 } , angular: { x: 0.0, y: 0.0, z: 0.0 } } , reference_frame: world } }'"
    os.system(cmd)

    print('The pose of robot:({},{}) '.format(x,y))
    return x, y


def savePointCloud(file_name, savepath, topic_name:str, folder:str = 'lidar'):
    """订阅点云并保存为npy"""
    # get_point
    msg = rospy.wait_for_message(topic_name, PointCloud2, timeout=None)

    ##  Get pointcloud from ros    
    point_cloud = pc2.read_points(msg, field_names=(
        "x", "y", "z", "intensity"))#, skip_nans=False

    savepath_folder = os.path.join(savepath, folder)
    if not os.path.exists(savepath_folder):
        os.makedirs(savepath_folder)
    
    # save_point
    np.save(os.path.join(savepath, folder ,file_name),np.array(list(point_cloud), dtype=np.float32)[:,:3]) #去除intensity
    print("{}/{} saved".format(folder,file_name))

    
    
def saveImage(img_file_name, savepath, topic_name:str, folder:str = 'depth_map'):
    """订阅img并保存为png"""
    # get_point
    msg = rospy.wait_for_message(topic_name, Image, timeout=None)
    
    np_img = ros_numpy.numpify(msg)
        
    # 使用PIL保存图像到文件
    image_pil = PILImage.fromarray(np_img)
    
    savepath_folder = os.path.join(savepath, folder)
    if not os.path.exists(savepath_folder):
        os.makedirs(savepath_folder)
    
    # save_img
    image_pil.save(os.path.join(savepath, folder ,img_file_name+'.png'))
    print("{}/{} saved".format(folder,img_file_name))




if __name__ == "__main__":
    
    rospy.init_node('get_points_once')

    local_time = str(time.strftime('%m%d%H%M', time.localtime()))
    savepath = '/home/wen/catkin_ws/src/p3at_gazebo/data_space/gazebo_lidar/' + local_time
    
    target_position = (11, 11)  # 目标位置

    # 距离目标的半径
    step = 0.5
    radius_array = np.arange(4,7,step)
    
    # 每次机器人生成的角度
    degree = 5
    angle_array = [i for i in range(0,181,degree)]
    
    # 按半径,角度递增的方式循环
    n = len(radius_array) * len(angle_array) # 进度条的总次数
    index = 0
    x_array, y_array, file_name_array= [], [], []
    for radius in radius_array:
        for angle_degree in angle_array:
            angle_radians = math.radians(angle_degree)
            
            # 瞬移机器人
            x, y = moveRobot(radius, angle_radians)
            x_array.append(x)   # 记录x坐标
            y_array.append(y)
            
            # 等待机器人静止
            time.sleep(0.5)
            
            # 命名点云保存文件
            index += 1
            file_name = 'point_' + str(index)
            file_name_array.append(file_name)   # 记录文件名
            
            # 保存点云, x,y,z
            savePointCloud(file_name, savepath, "/velodyne_points")
            savePointCloud(file_name, savepath, "/d455/camera/depth_registered/points",folder='depth_points')
            
            # 保存深度图和image
            img_file_name = 'image_' + str(index)
            saveImage(img_file_name, savepath, "/d455/camera/depth/image_raw")
            saveImage(img_file_name, savepath, "/d455/camera/color/image_raw", folder='img')
            
            # 进度条
            print('{0}/{1}'.format(index,n))
            
            # 检测中途是否停止了程序
            time.sleep(0.5)
            if rospy.is_shutdown():
                sys.exit("killed")
    print("Automove robot : Done")
    
    # txt保存, [lidar文件名, x, y]
    data = np.column_stack((x_array, y_array, file_name_array))
    output_path = os.path.join(savepath, "robot_coordiantes.txt")
    np.savetxt(output_path, data, delimiter=' ', fmt='%s')  # 数据分隔符为空格，以及格式为字符串格式%s
    print("log is saved :{}".format(output_path))
    
    