FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    wget lsb-release gnupg \
    libgl1 libgl1-mesa-dri libglu1-mesa \
    libglvnd0 libglx0 libegl1 \
    libxext6 libx11-6 libxrender1 libxi6 \
    x11-apps mesa-utils && \
    wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y gz-harmonic && \
    rm -rf /var/lib/apt/lists/*

# Create runtime directory for Qt/X11
RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root

WORKDIR /workspace

ENV QT_X11_NO_MITSHM=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root

CMD ["gz", "sim", "-g"]