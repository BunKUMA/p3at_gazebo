import open3d as o3d
import numpy as np
import os
import math
from drow_coordinates_iou import loadCoordinatesIouTxt
from iou_3d import get_3d_box, box3d_iou


target_h = 1.445
target_w = 1.879
target_l = 4.963

lidar_folders_path = '../data_space/gazebo_lidar_data'

file_list = np.array([i for i in os.listdir(lidar_folders_path)])

_ , sorted_data = loadCoordinatesIouTxt()

selected_iou = 0.3

selected_data = sorted_data[(sorted_data[:,2]>selected_iou) & (sorted_data[:,2]<selected_iou+0.1)]

random_data_index = np.random.choice(selected_data.shape[0], 1)
random_data = selected_data[random_data_index, :].squeeze()
x, y = random_data[0], random_data[1]
print(f"iou:{random_data[2]}")
str_x = str(x)+'0' if str(x)[-1] == '.' else str(x)
str_y = str(y)+'0' if str(y)[-1] == '.' else str(y)
file_name = 'point_'+str_x +'_'+str_y +'.npy'
print(file_name)



x_position_robot = math.sqrt((11-x)**2 + (11-y)**2)
y_position_robot = float(0)
z_position_robot = float(0)

theta = math.atan2(11-y, 11-x)
beta = np.pi - theta

yaw = beta + np.pi/2

# 计算正确的结果
corret_result = (x_position_robot, y_position_robot, z_position_robot, target_h, target_w, target_l, yaw)
# 加载预测的结果
folders_path = '../data_space/predictions'
file_path = os.path.join(folders_path,('predicte_'+str(file_name)))
predicte_result = np.load(file_path)

predicte_result = predicte_result[0]
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
max_indices = np.argmax(iou_arr, axis=0)
predicte = predicte_result[max_indices][1:-1].astype(np.float32)


# 创建点云
point_cloud = np.load(os.path.join(lidar_folders_path, file_name))
point_cloud = point_cloud[:,:3]
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_cloud)

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

# 创建自定义的三角形网格
faces = [
    [0, 1, 2],
    [0, 2, 3],
    [0, 4, 5],
    [0, 5, 1],
    [1, 5, 6],
    [1, 6, 2],
    [2, 6, 7],
    [2, 7, 3],
    [3, 7, 4],
    [3, 4, 0],
    [4, 7, 6],
    [4, 6, 5]
]

mesh = o3d.geometry.TriangleMesh()
mesh.vertices = o3d.utility.Vector3dVector(calculateBox(*corret_result))
mesh.triangles = o3d.utility.Vector3iVector(faces)

mesh2 = o3d.geometry.TriangleMesh()
mesh2.vertices = o3d.utility.Vector3dVector(calculateBox(*predicte))
mesh2.triangles = o3d.utility.Vector3iVector(faces)
mesh2.paint_uniform_color([1.0, 0.5, 0.0])  # 设置橙色

# 创建一个表示坐标系的网格对象
coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])

# 创建一个场景并将点云、包围框和坐标系添加到场景中
scene = o3d.visualization.Visualizer()
scene.create_window()
scene.add_geometry(pcd)        # 添加点云
scene.add_geometry(mesh)       # 添加包围框
scene.add_geometry(mesh2)       # 添加包围框
scene.add_geometry(coord_frame)  # 添加坐标系

# 设置视角
view_control = scene.get_view_control()
view_control.set_lookat([0, 0, 0])  # 设置视点
view_control.set_up([0, 0, 1])     # 设置视角上方的方向

# 显示场景
scene.run()
scene.destroy_window()
