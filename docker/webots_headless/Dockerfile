# Use the Webots nightly R2024a with correctly working Rosbot
FROM husarion/webots:humble

# Install TigerVNC server
RUN apt-get update && apt-get install -y \
    tigervnc-standalone-server \
    xfce4 \
    dbus-x11 \
    x11-xserver-utils \
    xfonts-base \
    xterm \
    && rm -rf /var/lib/apt/lists/*

# Copy and set up the startup script
COPY start.sh /start.sh
RUN chmod +x /start.sh

ENTRYPOINT ["/start.sh"]
