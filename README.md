# novel
Novel object detection, visualization and localization

This project was a class project for EN.530.707 Robot Systems Programming. 
All code is provided as is, and has no guarentee to actually work.

## Requirements

Hardware required:
* Turtlebot2
* RPLidar A2 mounted backwards on the middle ring
* Kinect camera mounted forwards
* (optional) Joystick


Packages required:
* turtlebot_navigation stack
* ar_track_alvar

## Installation

`cd ~/catkin_ws/src`

`git clone https://github.com/Raphtor/novel.git`

`cd .. && catkin_make`

## Running

1. First, make a map and save it

`roslaunch novel_bringup turtlebot_make_map.launch`


You can drive the turtlebot around using your favorite teleop package, or you can use the joystick with

`roslaunch novel_bringup turtlebot_make_map.launch use_controller:=true`

You then save the map file with 

`rosrun map_server map_saver -f your_map_name`

Note that your_map_name should NOT be a path, as this will screw up the yaml file.

2. Visualize novel objects in rviz without acting on them first. Note that your_map_file.yaml is now a path. This gives you a chance to dynamically reconfigure the lidar detection.

`roslaunch novel_bringup turtlebot_navigate.launch map_file:=your_map_file.yaml`

`rosrun rqt_reconfigure rqt_reconfigure`


3. Run the action node

`roslaunch action complete_action_real.launch map_file:=your_map_file.yaml`
