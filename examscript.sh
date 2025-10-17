#!/bin/bash
cd ros2_ws
colcon build
. install/local_setup.bash
source /opt/ros/humble/setup.bash    
source ~/ros2_ws/install/local_setup.bash
gnome-terminal -- ros2 launch examtask examtask_launch.py use_sim_time:=True #For starting the RViz and Gazebo   
gnome-terminal -- ros2 run twist_mux twist_mux --ros-args --params-file ./src/examtask/config/twist_mux.yaml -r  cmd_vel_out:=simple_diff_drive_controller/cmd_vel_unstamped #runs the twist_mux node with specific parameters and remaps the output topic cmd_vel_out to simple_diff_drive_controller/cmd_vel_unstamped
gnome-terminal -- ros2 launch nav2_bringup  navigation_launch.py  use_sim_time:=True #code launches the navigation_launch.py file from the nav2_bringup
gnome-terminal -- ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True #code launches the online_async_launch.py file from the slam_toolbox package 
gnome-terminal -- python3 src/examtask/scripts/Yellowobject.py  #For Starting Python Code for Finding the Yellow Object.
gnome-terminal -- ros2 launch explore_lite explore.launch.py use_sim_time:=True  #For Starting the Explore Lite for Navigating the Map.
