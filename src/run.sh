#!/bin/bash

apt update && apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    wget \
    x11-xserver-utils \
    mesa-utils \
    xserver-xorg-core \
    xserver-xorg-video-dummy \
    xinit \
    xauth \
    x11-xserver-utils

wget https://sourceforge.net/projects/virtualgl/files/2.6.5/virtualgl_2.6.5_amd64.deb/download -O virtualgl.deb &&
    apt install -y ./virtualgl.deb &&
    rm virtualgl.deb

cp /mcs_ws/src/xorg.conf /etc/X11/xorg.conf

Xorg :0 -config /etc/X11/xorg.conf &
export DISPLAY=:0
export XAUTHORITY=/root/.Xauthority

vglrun glxinfo | grep -i "OpenGL renderer"

# Init ROS 2 environment
source /opt/ros/humble/setup.bash
cd /mcs_ws
colcon build
source ./install/setup.bash

vglrun ros2 launch webots_ros2_cleverhive rosbot_launch.py use_headless:=true
