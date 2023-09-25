import numpy as np
import matplotlib.pyplot as plt 
from mpl_toolkits import mplot3d

def loadCoordinatesIouTxt():
    # 创建一个空列表来存储数据
    data = []

    # 打开文件以读取数据
    with open('robot_coordiantes.txt', 'r') as file:
        # 逐行读取文件内容
        for line in file:
        # 按照特定的格式解析每行数据
            parts = line.strip().split()
            # 将解析后的数据添加到列表中
            array = np.array(list(map(float,parts)))
            
            x, y= float(parts[0]), float(parts[1])
            radius = np.round(np.sqrt((11-x)**2 + (11-y)**2), decimals=1)
            # 存在半径的计算误差
            if (radius*10)%10 == 4 or (radius*10)%10 == 9: # 4.4m
                radius += 0.1
            if (radius*10)%10 == 6: # 4.6m
                radius -= 0.1
            
            data.append(np.append(array,radius))
    array = np.array(data)
    sorted_indices = np.argsort(array[:, 3])
    sorted_data = array[sorted_indices]

    unique_radius = np.unique(sorted_data[:,3])
    
    return unique_radius, sorted_data


if __name__ in "__main__":

    unique_radius, sorted_data = loadCoordinatesIouTxt()
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for radius in unique_radius:
        round_data = sorted_data[sorted_data[:,3]==radius]
        sorted_indices = np.argsort(round_data[:,1])
        round_data = round_data[sorted_indices]
        x = round_data[:,0]
        y = round_data[:,1]
        iou = [i * 10 for i in round_data[:,2]]
        ax.scatter(x, y, iou, label=f'radius:{radius}m')
        
    ax.set_xlabel('x coordinates') #设置x轴名称 x label
    ax.set_ylabel('y coordinates') #设置y轴名称 y label
    ax.set_zlabel('iou') #设置z轴名称 z label
    ax.legend() #自动检测要在图例中显示的元素，并且显示

    # 设置相同的刻度
    ax.set_aspect('equal')  # 设置刻度相同

    plt.show() #图形可视化
