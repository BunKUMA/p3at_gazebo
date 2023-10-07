#!/bin/bash
sourcefile="/home/wen/catkin_ws/src/p3at_gazebo/data_space/gazebo_lidar/suv/robot_coordiantes.txt"
destination_folder="/home/wen/catkin_ws/src/p3at_gazebo/data_space/voxel_sample"

#   列出文件夹的子文件夹
subfolders=("$destination_folder"/*/)

#   循环复制文件到所有子文件
for folder in "${subfolders[@]}"; do
    cp "$sourcefile" "$folder"
done