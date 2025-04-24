#!/bin/bash

apt-get update && apt-get install -y \
    tigervnc-standalone-server \
    xfce4 \
    dbus-x11 \
    x11-xserver-utils \
    xfonts-base \
    xterm

# Init ROS 2 environment
source /opt/ros/humble/setup.bash

# Create ~/.vnc directory if it doesn't exist
mkdir -p ~/.vnc

# Set VNC password from environment variable
if [ -n "$VNC_PASSWORD" ]; then
    echo "$VNC_PASSWORD" | vncpasswd -f >~/.vnc/passwd
    chmod 600 ~/.vnc/passwd
else
    echo "VNC_PASSWORD environment variable is not set"
    exit 1
fi

# Start VNC server
/usr/bin/vncserver $VNC_DISPLAY \
    -geometry 1920x1080 \
    -depth 24 \
    -localhost no

cd /mcs_ws

colcon build

source ./install/setup.bash

export DISPLAY=$VNC_DISPLAY

export

tail -f /dev/null
# DISPLAY=$VNC_DISPLAY ros2 launch webots_ros2_cleverhive rosbot_launch.py use_headless:=true
