Launch Files When Testing

roslaunch novel_gazebo turtlebot_world.launch

roslaunch turtlebot_rviz_launchers view_navigation.launch --screen

roslaunch turtlebot_gazebo amcl_demo.launch map_file:=<path to map yaml file>

roslaunch detect_aruco kinect_alvar.launch

roslaunch action action_node.launch 
