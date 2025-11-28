FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------
# Install ROS 2 Jazzy + Foxglove Bridge
# -----------------------------
RUN apt-get update && apt-get install -y \
    locales curl gnupg lsb-release \
    software-properties-common \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && add-apt-repository universe

# Install ROS 2 apt source (combined into single RUN to preserve variable)
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
 && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" \
 && dpkg -i /tmp/ros2-apt-source.deb \
 && rm /tmp/ros2-apt-source.deb

RUN apt update && apt install -y ros-dev-tools

RUN apt update && apt upgrade -y && apt install -y ros-jazzy-ros-base

RUN apt-get update && apt-get install -y \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-ros-gz 

# -----------------------------
# Install Gazebo Harmonic
# -----------------------------
RUN apt-get update \
 && apt-get install -y wget \
 && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
      http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y gz-harmonic
# -----------------------------
# Setup environment
# -----------------------------
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/gz/setup.sh" >> ~/.bashrc

WORKDIR /workspace
# Gazebo Socket default
EXPOSE 11345
# Foxglove WebSocket default
EXPOSE 8765   

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]