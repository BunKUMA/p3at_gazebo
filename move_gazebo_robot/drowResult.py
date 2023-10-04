import os 
import numpy as np
import matplotlib.pyplot as plt 
from mpl_toolkits import mplot3d

def loadIou(path):
    # 创建一个空列表来存储数据
    data = []
    # 打开文件以读取数据
    with open(path, 'r') as file:
        # 逐行读取文件内容
        for line in file:
        # 按照特定的格式解析每行数据
            parts = line.strip().split()
            # 将解析后的数据
            iou= float(parts[1])
            data.append(iou)
    return np.array(data)


def loadCoordiantes(iou_arr, path):
    # 创建一个空列表来存储数据
    data = []

    # 打开文件以读取数据
    with open(path, 'r') as file:
        # 逐行读取文件内容
        for line in file:
        # 按照特定的格式解析每行数据
            parts = line.strip().split()
            # 将解析后的数据添加到列表中
            array = np.array(list(map(float,parts[:2])))
            
            x, y= float(parts[0]), float(parts[1])
            radius = np.round(np.sqrt((11-x)**2 + (11-y)**2), decimals=1)
            # 存在半径的计算误差
            if (radius*10)%10 == 4 or (radius*10)%10 == 9: # 4.4m
                radius += 0.1
            if (radius*10)%10 == 6: # 4.6m
                radius -= 0.1
            
            data.append(np.append(array,radius))
    
    array = np.hstack((np.array(data), iou_arr[:,np.newaxis]))
    

    return array


    
# folders_path = '/home/wen/catkin_ws/src/p3at_gazebo/data_space/gazebo_lidar/volvoS90'
def drowResult(folders_path):
    iou_path = os.path.join(folders_path,'iou.txt')
    coordiantes_path = os.path.join(folders_path,'robot_coordiantes.txt')
    
    iou_arr = loadIou(iou_path)
    
    data = loadCoordiantes(iou_arr, coordiantes_path)
    
    condition1 = data[:,3] == 0.0    #   iou==0.0
    condition2 = (data[:,3] > 0.0) & (data[:,3] < 0.3)
    condition3 = data[:,3] >= 0.3 
    conditions = [condition1, condition2, condition3]
    colors = ['red','orange','green']
    labels = ['non-detectable', 'hard-detectable', 'detectable']
    
    fig = plt.figure()
    ax = fig.add_subplot()

    for condition, color, label in zip(conditions, colors,labels):
        round_data = data[condition]
        x = round_data[:,0]
        y = round_data[:,1]
        ax.scatter(x, y, c=color,label=label)
        
    ax.set_xlabel('x coordinates') #设置x轴名称 x label
    ax.set_ylabel('y coordinates') #设置y轴名称 y label
    # ax.set_zlabel('iou') #设置z轴名称 z label
    ax.legend(loc='upper right',bbox_to_anchor=(2.0,1.0)) #自动检测要在图例中显示的元素，并且显示

    # 设置相同的刻度
    ax.set_aspect('equal')  # 设置刻度相同

    output = os.path.join(folders_path,'result_img')
    os.makedirs(output, exist_ok=True)
        
    plt.savefig(os.path.join(output,'result.png'))
    # plt.show() #图形可视化

if __name__ == "__main__":
    pass