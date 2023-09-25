## 流程

1. mobe_robot_in_gazebo.py

   将产生lidar文件夹和robot_coordiantes.txt

   ​	lidar/point_*.npy : 点云

   ​	robot_coordiantes.txt : 机器人坐标, 点云文件名

   ​	机器人坐标: x y 

2. voxelnet

   将产生predictions文件夹

   ​	predictions/predicte_point_*.npy : 预测结果

   ​	预测结果 : 

3. createLabel.py

   将产生label.txt

   ​	label.txt : 点云文件名 x y z h w l yaw

4. evaluate_iou.py

   产生iou.txt

   ​	iou.txt : 预测文件名 iou

5. drow_......