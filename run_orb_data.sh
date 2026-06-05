#!/bin/bash

# 第一个终端：启动 ROS launch
gnome-terminal -- bash -c "
cd ~/catkin_ws;
source ~/catkin_ws/devel/setup.bash;
roslaunch orb_slam3_ros euroc_mono_inertial.launch;
exec bash"

# 等待 2 秒再启动第二个终端
sleep 2

# 第二个终端：播放 rosbag
gnome-terminal -- bash -c "
source ~/catkin_ws/devel/setup.bash;
cd ~/catkin_ws/datasets;
rosparam set use_sim_time true;
rosbag play MH_01_easy.bag --clock -r 0.8;
exec bash"

