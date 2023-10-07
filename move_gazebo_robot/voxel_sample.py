import os
import numpy as np
import open3d as o3d
from tqdm import tqdm


# 定义文件夹路径
project_path = '/home/wen/catkin_ws/src/p3at_gazebo/data_space/gazebo_lidar'
project_names = ['suv']
points_folders = ['depth_points']
output_project = '/home/wen/catkin_ws/src/p3at_gazebo/data_space/voxel_sample'
os.makedirs(output_project,exist_ok=True)


#   文件夹层级
for project_name in project_names:
    
    folder_path = os.path.join(project_path, project_name)
    
    #   点云层级
    for points_folder_name in points_folders:
        
        input_points_files_name = [file_name for file_name in os.listdir(os.path.join(folder_path, points_folder_name))]
        
        for i in tqdm(np.arange(0.01,1,0.01)):
            
            #   创建输出文件夹
            output_path = os.path.join(output_project , str(round(i,2)), points_folder_name)
            os.makedirs(output_path,exist_ok=True)
            
            #   点云文件
            for input_points_file_name in input_points_files_name:
                point_cloud = np.load(os.path.join(folder_path, points_folder_name,input_points_file_name))
            
                sparse_cloud = o3d.geometry.PointCloud()
                sparse_cloud.points = o3d.utility.Vector3dVector(point_cloud)
                # print("点云：", sparse_cloud)
                # ===========================================================
                # ------------------------- 稀疏化 -------------------------
                voxel_size = i
                voxel_cloud = sparse_cloud.voxel_down_sample(voxel_size)
                output_cloud = np.asarray(voxel_cloud.points)
                np.save(os.path.join(output_path, input_points_file_name),output_cloud)