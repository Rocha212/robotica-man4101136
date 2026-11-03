#!/bin/bash

echo "=== 1. Sistema operativo ==="
lsb_release -d
echo

echo "=== 2. ROS 2 ==="
echo "ROS_DISTRO: $ROS_DISTRO"
dpkg -l ros-jazzy-ros-base | tail -1 | awk '{print "ros-base:", $3}'
echo

echo "=== 3. Python y librerías ==="
python3 --version
python -c "import numpy, matplotlib; print('numpy', numpy.__version__, '| matplotlib', matplotlib.__version__)"
python -c "from roboticstoolbox import Bicycle, EKF, RandomPath, LandmarkMap, RangeBearingSensor; print('roboticstoolbox: importación OK')"
echo

echo "=== 4. Paquetes de la parte 5 ==="
dpkg -l | grep -E "turtlebot3-gazebo|ros-gz-sim|cartographer-ros|navigation2" | awk '{print $2, $3}'
gz sim --version | head -1
echo

echo "=== 5. Archivos de configuración de Cartographer ==="
ls /opt/ros/jazzy/share/cartographer/configuration_files/ | grep -E "map_builder.lua|trajectory_builder.lua|pose_graph.lua"
EOF
