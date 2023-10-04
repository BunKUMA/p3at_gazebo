import os
import math
import numpy as np
from evaluate_iou import evaluate_iou
from drowResult import drowResult

def createLabel(folders_path, target_h, target_w, target_l, x, y):
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

       
if __name__ == '__main__':
    
    
    # 车的数据
    car = { 'suv':{'h':1.684,
                'w':1.843,
                'l':4.646}, 
           
            'benzc63':{'h':1.447,
                    'w':1.795,
                    'l':4.726},

            'audiA7':{'h':1.422,
                    'w':1.908,
                    'l':4.969},

            'dodoge':{'h':1.420,
                    'w':1.930,
                    'l':5.030}, 

            'volvoS90':{'h':1.445,
                    'w':1.879,
                    'l':4.963}}
    
    target_position = (11,11)
    x = target_position[0]
    y = target_position[1]
    
    project_path = '/home/wen/catkin_ws/src/p3at_gazebo/data_space/gazebo_lidar'
    projects_name = os.listdir(project_path)
    
    for project_name in projects_name:
        if project_name not in car:
            # raise KeyError(f"The data of '{project_name}' does not exist")
            project_name = 'suv'
        target_h = car[project_name]['h']
        target_w = car[project_name]['w']
        target_l = car[project_name]['l']
        folders_path = os.path.join(project_path,project_name)
        createLabel(folders_path, target_h, target_w, target_l, x, y)
        evaluate_iou(folders_path)
        drowResult(folders_path)
        print(f'{project_name}:done')
        
        
    
    