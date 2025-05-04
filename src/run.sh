#!/bin/bash

# apt-get update && apt-get install -y \
#     tigervnc-standalone-server \
#     xfce4 \
#     dbus-x11 \
#     x11-xserver-utils \
#     xfonts-base \
#     xauth \
#     xterm

apt update && apt install -y \
    curl gnupg2 lsb-release wget sudo x11-xserver-utils mesa-utils

apt install -y \
    xserver-xorg-core xserver-xorg-video-dummy \
    xinit xauth x11-xserver-utils

wget https://sourceforge.net/projects/virtualgl/files/2.6.5/virtualgl_2.6.5_amd64.deb/download -O virtualgl.deb &&
    apt install -y ./virtualgl.deb &&
    rm virtualgl.deb

# wget https://github.com/TurboVNC/turbovnc/releases/download/3.2beta1/turbovnc_3.1.90_amd64.deb -O turbovnc.deb &&
#     apt install -y ./turbovnc.deb &&
#     rm turbovnc.deb

cp /mcs_ws/src/xorg.conf /etc/X11/xorg.conf

# Set password for VNC
# mkdir -p /root/.vnc &&
#     echo "mypassword" | /opt/TurboVNC/bin/vncpasswd -f >/root/.vnc/passwd &&
#     chmod 600 /root/.vnc/passwd

Xorg :0 -config /etc/X11/xorg.conf &
export DISPLAY=:0
export XAUTHORITY=/root/.Xauthority

# Run VirtualGL server config
# /opt/VirtualGL/bin/vglserver_config -config +s +f +t

# /opt/TurboVNC/bin/vncserver :0 -geometry 1920x1080 -depth 24

# export DISPLAY=:0

vglrun glxinfo | grep -i "OpenGL renderer"

# Init ROS 2 environment
source /opt/ros/humble/setup.bash

# Create ~/.vnc directory if it doesn't exist
# mkdir -p ~/.vnc

# Set VNC password from environment variable
# if [ -n "$VNC_PASSWORD" ]; then
#     echo "$VNC_PASSWORD" | vncpasswd -f >~/.vnc/passwd
#     chmod 600 ~/.vnc/passwd
# else
#     echo "VNC_PASSWORD environment variable is not set"
#     exit 1
# fi

# # Start VNC server
# /usr/bin/vncserver $VNC_DISPLAY \
#     -geometry 1920x1080 \
#     -depth 24 \
#     -localhost no

cd /mcs_ws

colcon build

source ./install/setup.bash

export DISPLAY=$VNC_DISPLAY

export

tail -f /dev/null
# DISPLAY=$VNC_DISPLAY ros2 launch webots_ros2_cleverhive rosbot_launch.py use_headless:=true
