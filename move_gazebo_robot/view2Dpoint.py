import os 
import  numpy as np
import matplotlib.pyplot as plt


def draw_point_cloud(ax, points, axes=[0, 1],point_size=1.0,color='blue'):
        """
        draw_point_cloud in the matplot
        Convenient method for drawing various point cloud projections as a part of frame statistics.
        """
        axes_str = ['X','Y']
        ax.grid(False)
        
        ax.scatter(*np.transpose(points[:, axes]), s=point_size, c=color)


def calculateBox(x,y,z,h,w,l,yaw):
    half_h, half_w, half_l = h / 2.0, w / 2.0, l / 2.0
    vertices = [
        [-half_l, -half_w, -half_h],
        [half_l, -half_w, -half_h],
        [half_l, half_w, -half_h],
        [-half_l, half_w, -half_h],
        [-half_l, -half_w, half_h],
        [half_l, -half_w, half_h],
        [half_l, half_w, half_h],
        [-half_l, half_w, half_h]
    ]

    # 应用偏航角
    R = np.array([[np.cos(yaw), -np.sin(yaw), 0.0],
                [np.sin(yaw), np.cos(yaw), 0.0],
                [0.0, 0.0, 1.0]])
    vertices = [np.dot(R, np.array(vertex)) + np.array([x, y, z]) for vertex in vertices]

    return vertices

def vertices3d_to2d(vertices:list):
    vertices = [vertex[:2] for vertex in vertices]
    return np.array(vertices)


def createBox(ax, corret_result,edgecolor,linewidth):
    # 创建自定义的三角形网格
    polygon = plt.Polygon(corret_result, fill=False, edgecolor=edgecolor, linewidth=linewidth) # type: ignore
    ax.add_patch(polygon)




if __name__ == '__main__':
    # 定义文件夹路径
    project_path = '/home/wen/catkin_ws/src/p3at_gazebo/data_space/gazebo_lidar'
    project_names = [folder for folder in os.listdir(project_path)]
    target_folders = ['lidar', 'depth_points']
    
    
    
    #   project_name:车的名字
    for project_name in project_names:
        if project_name == 'test':
            continue
        
        #   车的文件夹
        folder_path = os.path.join(project_path, project_name)
        output_path, predicte_folder_path = '', ''
        
        #   进入点云或者深度点云文件夹
        for target_folder_name in target_folders:
            if target_folder_name=='lidar':
                output_path = os.path.join(folder_path,'img_lidar')
                predicte_folder_path = os.path.join(folder_path,'predictions')
            else:
                output_path = os.path.join(folder_path,'img_depth')
                predicte_folder_path = os.path.join(folder_path,'predictions_depthmap')
            
            
            if os.path.exists(output_path):
                print(f'exist: {project_name},{target_folder_name}')
                # continue
            
            os.makedirs(output_path,exist_ok=True)
            
            #   点云文件夹
            target_folder_path = os.path.join(folder_path, target_folder_name)
            
            with open(os.path.join(folder_path,'label.txt'), 'r') as labels:
                # 每行遍历标签
                for line in labels:
                    parts = line.strip().split()    # 去除回车和换行符
                    target_name, corret_result = parts[0], np.array(parts[1:],dtype=np.float32)
                
                    #   加载预测的结果
                    predicte_file_name = 'predicte_'+ target_name + '.npy'
                    predicte_file_path = os.path.join(predicte_folder_path ,predicte_file_name)
                    predicte_result = np.load(predicte_file_path)
                    predicte_result = predicte_result[0]
                    
                    #   加载点云
                    target_file_path = os.path.join(target_folder_path, target_name + '.npy')
                    points = np.load(target_file_path)
                    
                
                    fig,ax = plt.subplots()
                    #   画点云俯视图,没3个点画一次
                    draw_point_cloud(ax,points[::3],axes=[0,1],point_size=0.01,color='gray')
                    
                    #   画正确的结果box
                    vertices = calculateBox(*corret_result)
                    vertices = vertices3d_to2d(vertices)
                    createBox(ax, vertices,edgecolor='b',linewidth=0.5)
                    
                    #   画预测结果的box俯视图
                    for result in predicte_result:
                        result = result[1:8].astype(np.float32)
                        vertices = calculateBox(*result)
                        vertices = vertices3d_to2d(vertices)
                        createBox(ax, vertices,edgecolor='r',linewidth=0.5)

                    # 去掉x轴和y轴的刻度
                    ax.set_xticks([])
                    ax.set_yticks([])

                    # 去掉x轴和y轴的框（轴线）
                    ax.spines['top'].set_visible(False)
                    ax.spines['right'].set_visible(False)
                    ax.spines['bottom'].set_visible(False)
                    ax.spines['left'].set_visible(False)   
                    plt.gca().set_aspect('equal', adjustable='box')  # 设置坐标轴比例为1:1，确保正方形显示
                    plt.savefig(os.path.join(output_path,target_name+'.png'))
                    plt.close()