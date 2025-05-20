#!/bin/bash

# Install transitiverobotics
echo ============= TRANSITIVE SETUP =============
# Try to get credentials from environment variables first, then from secrets
if [ -n "$TRANSITIVE_ID" ] && [ -n "$TRANSITIVE_TOKEN" ]; then
  # Use environment variables if available
  curl -sf "https://install.transitiverobotics.com?id=${TRANSITIVE_ID}&token=${TRANSITIVE_TOKEN}&docker=true" | bash
else
  echo "Error: Transitive credentials not found in environment"
  exit 1
fi

# Bootstrap the Transitive agent
if [ ! -e "$HOME/.transitive/.installation_complete" ]; then
  mkdir -p "$HOME/.transitive"
  cp -r /transitive-preinstalled/. "$HOME/.transitive"
  rm -rf /transitive-preinstalled
fi
cd $HOME/.transitive # NOTE: prevents from ENOENT certs/client.crt error
bash $HOME/.transitive/start_agent.sh &
echo ========================================

# Sanity check DISPLAY environment variable
echo ============= DISPLAY INFO =============
echo Display is: $DISPLAY
echo ========================================

# Print GPU info
echo ============= GPU INFO =============
nvidia-smi
echo ========================================

# Get the GPU bus ID in correct format for xorg.conf
echo ============= GPU BUS ID =============
busid=$(nvidia-smi --query-gpu=pci.bus_id --format=csv,noheader,nounits | head -n1)
# Convert hex bus (e.g., 82:00.0) to decimal:device:function
IFS=":."
read -r hexbus device func <<<"${busid#*:}"
bus=$((16#$hexbus))
xorg_busid="PCI:${bus}:${device}:${func}"
echo "Xorg BusID: $xorg_busid"
echo ========================================

# Patch xorg.conf in place
sed -i "s/^ *BusID *\"PCI:[0-9:]*\"/    BusID          \"$xorg_busid\"/" /etc/X11/xorg.conf
echo "✅ Updated Xorg BusID to: $xorg_busid"
cat /etc/X11/xorg.conf
echo ========================================

# Start X server for webots to render to
Xorg :99 -config /etc/X11/xorg.conf &
sleep 2

# Print OpenGL renderer info
echo ============= GPU DEBUG INFO =============
vglrun -d :99 glxinfo | grep -i 'OpenGL renderer'
echo ========================================

# Source ROS and run the application
source /opt/ros/humble/setup.bash
source /mcs_ws/install/setup.bash

echo ============= RUNNING WEBOTS =============
vglrun -d :99 ros2 launch webots_ros2_cleverhive rosbot_launch.py use_headless:=true
