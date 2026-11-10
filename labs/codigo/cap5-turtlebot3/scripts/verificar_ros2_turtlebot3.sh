#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
echo 'ROS_DISTRO:' \${ROS_DISTRO:-no definido}
echo 'Paquetes TurtleBot3:'
ros2 pkg list | grep turtlebot3 || true
echo 'Tópicos activos:'
ros2 topic list
echo 'Tipo de /cmd_vel (si existe):'
ros2 topic type /cmd_vel || true
echo 'Tipo de /odom (si existe):'
ros2 topic type /odom || true
