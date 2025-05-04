FROM husarion/webots:humble

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt update && apt install -y \
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

# Install VirtualGL
RUN wget https://sourceforge.net/projects/virtualgl/files/2.6.5/virtualgl_2.6.5_amd64.deb/download -O /tmp/virtualgl.deb && \
    apt install -y /tmp/virtualgl.deb && rm /tmp/virtualgl.deb

# Copy your project source and Xorg config
COPY ./src /mcs_ws/src
COPY ./config/xorg.conf /etc/X11/xorg.conf

# Set environment variables
ENV DISPLAY=:0
ENV XAUTHORITY=/root/.Xauthority
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

WORKDIR /mcs_ws

# Build the project
RUN source /opt/ros/humble/setup.bash && \
    colcon build

# Default command
CMD bash -c "\
    Xorg :0 -config /etc/X11/xorg.conf & \
    sleep 2 && \
    export DISPLAY=:0 && \
    export XAUTHORITY=/root/.Xauthority && \
    vglrun glxinfo | grep -i 'OpenGL renderer' && \
    source /opt/ros/humble/setup.bash && \
    source ./install/setup.bash && \
    vglrun ros2 launch webots_ros2_cleverhive rosbot_launch.py use_headless:=true"
