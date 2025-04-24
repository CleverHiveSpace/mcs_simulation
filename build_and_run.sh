#! /bin/bash

# Install vncserver
apt update
apt install -y tigervnc-standalone-server

# Go to the workspace
cd /mcs_ws

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Run webots in fake headless mode by setting the DISPLAY variable
export DISPLAY=:0
ros2 launch webots_ros2_cleverhive ${ROBOT_NAME:-rosbot}_launch.py
