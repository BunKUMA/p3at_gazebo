import os
import numpy as np
from iou_3d import get_3d_box, box3d_iou


def evaluate_iou(predicte_result, corret_result):
    iou_arr = []
    # 多个预测结果选最好的
    for predicte in predicte_result:
        if len(predicte) == 0:
            break
        predicte = predicte[1:-1].astype(np.float32)
        
        x1, y1, z1, h1, w1, l1, yaw1 = corret_result
        x2, y2, z2, h2, w2, l2, yaw2 = predicte
        
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
        
    return iou



if __name__ == "__main__":
    folders_path = '/home/wen/catkin_ws/src/p3at_gazebo/data_space/gazebo_lidar/09251722'
    
    predicte_file_name_arr , iou_arr = [], []
    with open(os.path.join(folders_path,'label.txt')) as labels:
        # 每行遍历标签
        for line in labels:
            parts = line.strip().split()    # 去除回车和换行符
            lidar_file_name, corret_result = parts[0], np.array(parts[1:],dtype=np.float32)
            
            # 加载预测的结果
            predicte_file_name = 'predicte_'+lidar_file_name 
            predicte_file_path = os.path.join(folders_path, 'predictions',predicte_file_name + '.npy')
            predicte_result = np.load(predicte_file_path)
            predicte_result = predicte_result[0]
            
            # 计算iou
            iou = evaluate_iou(predicte_result, corret_result)
            
            # 记录预测结果文件名和iou
            iou_arr.append(iou)
            predicte_file_name_arr.append(predicte_file_name)
    
    # 保存在txt
    data = np.column_stack((predicte_file_name_arr, iou_arr))
    output_path = os.path.join(folders_path,'iou.txt')
    np.savetxt(output_path, data, delimiter=' ', fmt='%s')  # 数据分隔符为空格，以及格式为字符串格式%s
    
    print('Done')