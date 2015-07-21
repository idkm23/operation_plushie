#!/bin/bash
echo "In order to source properly, run this script using '. ./<script>'!"
source ~/indigo/rethink_ws/devel/setup.bash
exec "$@"
export ROS_MASTER_URI=http://baxter.local:11311
export ROS_IP=10.0.7.114   
exec "$@"                                         
echo "Done with sourcing and environment."
rosrun baxter_tools enable_robot.py -e
echo "Enabled robot."
rosrun baxter_tools camera_control.py -c left_hand_camera
echo "Closed left cam."
rosrun baxter_tools camera_control.py -c right_hand_camera
echo "Closed right cam."
rosrun baxter_tools camera_control.py -c head_camera
echo "Closed head cam."
rosrun baxter_tools camera_control.py -o left_hand_camera -r 640x400
echo "Opened left cam."
rosrun baxter_tools camera_control.py -c right_hand_camera
echo "Closed right cam again."
rosrun baxter_tools camera_control.py -o head_camera -r 960x600
echo "Opened head cam."
source ~/indigo/Baxter_ws/devel/setup.bash
exec "$@"
