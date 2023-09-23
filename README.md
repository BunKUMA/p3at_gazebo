# p3at_gazebo
Using the p3at bot in gazebo

![](https://github.com/BunKUMA/p3at_gazebo/blob/main/Screenshot.png)

## starting in the following :

```bash
cd ~/catkin_ws
source devel/setup.bash 
roslaunch p3at_description velodyne_and_robot.launch
```

## tips

when camera has no signal

```bash
sudo apt-get install ros-melodic-gazebo-ros-pkgs
```

using the keyboard to control the robot

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot/cmd_vel
```

