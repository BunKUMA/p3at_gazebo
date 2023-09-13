import open3d as o3d
import numpy as np

# 创建一个空的点云
pcd = o3d.geometry.PointCloud()

# 填充点云数据（假设你有点云数据）

# 创建一个表示框的网格对象
x, y, z = 1.0, 2.0, 3.0  # 框的中心坐标 (x, y, z)
h, w, l = 0.5, 0.3, 0.2  # 框的高度、宽度和长度
yaw = 45.0  # 框的偏航角（度数）

box = o3d.geometry.TriangleMesh.create_box(width=w, height=h, depth=l)
rotation = np.radians(yaw)
R = np.array([[np.cos(rotation), -np.sin(rotation), 0.0],
              [np.sin(rotation), np.cos(rotation), 0.0],
              [0.0, 0.0, 1.0]])
T = np.array([[1.0, 0.0, 0.0, x],
              [0.0, 1.0, 0.0, y],
              [0.0, 0.0, 1.0, z],
              [0.0, 0.0, 0.0, 1.0]])
box.transform(np.dot(T, R))

# 创建一个场景并将点云和框添加到场景中
scene = o3d.visualization.Visualizer()
scene.create_window()
scene.add_geometry(pcd)  # 添加点云
scene.add_geometry(box)  # 添加框

# 设置视角
view_control = scene.get_view_control()
view_control.set_lookat([0, 0, 0])  # 设置视点
view_control.set_up([0, 0, 1])     # 设置视角上方的方向

# 显示场景
scene.run()
scene.destroy_window()
