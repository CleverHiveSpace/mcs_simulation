FROM husarion/webots:humble

ENV DEBIAN_FRONTEND=noninteractive

# Fix ROS2 keys after may 2025 update
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list


# Install dependencies
RUN apt update && apt install -y \
    build-essential \
    curl \
    git \
    gnupg \
    gnupg2 \
    lsb-release \
    wget \
    x11-xserver-utils \
    mesa-utils \
    xserver-xorg-core \
    xserver-xorg-video-dummy \
    xinit \
    xauth \
    x11-xserver-utils \
    ros-humble-rosbridge-server

# Install VirtualGL
RUN wget https://sourceforge.net/projects/virtualgl/files/2.6.5/virtualgl_2.6.5_amd64.deb/download -O /tmp/virtualgl.deb && \
    apt install -y /tmp/virtualgl.deb && rm /tmp/virtualgl.deb

# Copy your project source and Xorg config
COPY ./src /mcs_ws/src
COPY ./config/xorg.conf /etc/X11/xorg.conf

# Set environment variables
ENV DISPLAY=:99
ENV XAUTHORITY=/root/.Xauthority
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

WORKDIR /mcs_ws

# Build the project
RUN source /opt/ros/humble/setup.bash && \
    colcon build

# Build the remote control server
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt install -y nodejs && \
    cd /mcs_ws/src/mcs_remote_control && \
    npm install && \
    npm run build

# Copy the entrypoint script
COPY ./entrypoint.sh /mcs_ws/entrypoint.sh

# Make entrypoint script executable
RUN chmod +x /mcs_ws/entrypoint.sh

# Use the entrypoint script
CMD ["/mcs_ws/entrypoint.sh"]
