#!/bin/bash

# ========================= ROS 2 =========================
# Init ROS 2 environment
source /opt/ros/humble/setup.bash

# ========================= VNC server =========================
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
/usr/bin/vncserver :0 \
    -geometry 1920x1080 \
    -depth 24 \
    -localhost no

# Set the display to :0
export DISPLAY=:11

# Run the next command
exec "$@"
