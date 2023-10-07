from function import Function, PointFunction
from PIL import Image
import numpy as np
import os
from tqdm import tqdm

DEPTH_SCALE_M = 0.001
# 以下示例假设相机参数为 fx, fy（焦距），cx, cy（光学中心）
fx = 392.323 
fy = 392.323
cx = 319.158 
cy = 240.853

project_path = '/home/wen/catkin_ws/src/p3at_gazebo/data_space/process_height'
folders_name = [file for file in os.listdir(project_path)]

for folder_name in folders_name:
    output_path = project_path +'/'+ folder_name + '/depth_points'
    folder_path = project_path +'/'+ folder_name + '/depth_map'
    

    files_name = [file for file in os.listdir(folder_path)]
    for file_name in tqdm(files_name):
        file_path = os.path.join(folder_path,file_name)

        depth_image = Image.open(file_path)
        depth_data = np.array(depth_image)
        depth_data = Function(depth_data)
        # 创建一个空的点云列表，用于存储点的坐标
        point_cloud = []

        # 遍历深度图像的每个像素
        for y in range(depth_data.shape[0]):
            for x in range(depth_data.shape[1]):
                # 获取深度值
                depth = depth_data[y, x] * DEPTH_SCALE_M

                # 计算对应的点的坐标（假设 Z 轴为深度，X 和 Y 轴为图像平面）
                if depth > 0:
                    point_x = (x - cx) * depth / fx
                    point_y = (y - cy) * depth / fy
                    point_z = depth 

                    # 将点的坐标添加到点云列表中
                    point_cloud.append([point_x, point_y, point_z])
        # 绕 Y 轴顺时针旋转 90 度
        angle_y = np.radians(90)  # 将角度转换为弧度
        rotation_y = np.array([[np.cos(angle_y), 0, np.sin(angle_y)],
                            [0, 1, 0],
                            [-np.sin(angle_y), 0, np.cos(angle_y)]])
        rotated_points = np.dot(point_cloud, rotation_y.T)
        # 绕 X 轴逆时针旋转 90 度
        angle_x = np.radians(-90)  # 将角度转换为弧度
        rotation_x = np.array([[1, 0, 0],
                            [0, np.cos(angle_x), -np.sin(angle_x)],
                            [0, np.sin(angle_x), np.cos(angle_x)]])
        rotated_points = np.dot(rotated_points, rotation_x.T)

        nan_mask = np.isnan(rotated_points).any(axis=1)
        point_cloud = rotated_points[~nan_mask]
        # 步骤 4：可选地，保存点云数据到文件或进行可视化
        # 这里不提供具体的点云保存或可视化代码，因为涉及到不同的库和工具。

        # 将点云列表转换为 NumPy 数组
        point_cloud = np.array(point_cloud)
        
        point_cloud = PointFunction(point_cloud)
        
        root, extension = os.path.splitext(file_name)
        np.save(os.path.join(output_path,'point_'+root[6:]+'.npy'), point_cloud)
    