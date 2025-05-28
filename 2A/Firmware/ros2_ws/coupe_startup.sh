#!/bin/bash
ROS_WORKSPACE=/home/carpe-bleue/ros2_ws
cd $ROS_WORKSPACE

source /opt/ros/humble/setup.bash
source install/setup.bash
sudo chmod 666 /dev/serial0

ros2 launch ydlidar_ros2_driver ydlidar_launch.py &
ros2 run ydlidar_ros2_driver safety_node.py &
ros2 run serial_package screen_reader.py &
ros2 run serial_package tirette &
ros2 run serial_package bag_listener.py &
ros2 run serial_package emergency_stop.py &
ros2 run serial_package merged_node.py

wait
