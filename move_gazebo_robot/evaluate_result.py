import os
import math
import numpy as np
from iou_3d import get_3d_box, box3d_iou

folders_path = '/home/wen/workspace/predictions'

file_list = [i for i in os.listdir(folders_path) if i.endswith('.npy')]

target_position = (11,11)
x = target_position[0]
y = target_position[1]
target_h = 1.445
target_w = 1.879
target_l = 4.963
original_yaw_degrees = -90

with open('coordinates_iou.txt', 'w') as file:
    # 遍历文件夹里所有文件
    for i, file_name in enumerate(file_list):
        position_str = file_name[15:] #11.7_7.1
        robot_x = float(position_str[:4])
        robot_y = float(position_str[5:-4])

        x_position_robot = math.sqrt((x-robot_x)**2 + (y-robot_y)**2)
        y_position_robot = float(0)
        z_position_robot = float(0)

        theta = math.atan2(y-robot_y, x-robot_x)
        beta = np.pi - theta
        
        yaw = beta + np.pi/2

        # 计算正确的结果
        corret_result = (x_position_robot, y_position_robot, z_position_robot, target_h, target_w, target_l, yaw)
        
        # 加载预测的结果
        file_path = os.path.join(folders_path,file_name)
        predicte_result = np.load(file_path)
        
        predicte_result = predicte_result[0]
        iou_arr = []
        # 多个预测结果选最好的
        for predicte in predicte_result:
            if len(predicte) == 0:
                break
            predicte = predicte[1:-1].astype(np.float32)
        
            h1,w1,l1 = corret_result[3],corret_result[4],corret_result[5]
            h2,w2,l2 = predicte[3],predicte[4],predicte[5]
            yaw1 = corret_result[6]
            yaw2 = predicte[6]
            x1,y1,z1 = corret_result[0],corret_result[1],corret_result[2]
            x2,y2,z2 = predicte[0],predicte[1],predicte[2]

            # 计算3d iou
            # get_3d_box(box_size(length,wide,height), heading_angle, center)
            corners_3d_ground  = get_3d_box((l1,w1,h1), yaw1, (x1,y1,z1)) 
            corners_3d_predict = get_3d_box((l2,w2,h2), yaw2, (x2,y2,z2))
            iou_3d,_ = box3d_iou(corners_3d_predict,corners_3d_ground)
            iou_arr.append(iou_3d)
            
        if iou_arr == []:
            iou =0.0
        else:
            iou = max(iou_arr)
        
        file.write(f"{robot_x} {robot_y} {iou}\n")
        print(f'{i+1}/{len(file_list)}')