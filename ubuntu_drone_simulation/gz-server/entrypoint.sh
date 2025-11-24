#!/bin/bash
set -e

echo "=== Starting Gazebo Server Setup ==="

source /opt/ros/jazzy/setup.bash
echo "✓ ROS 2 sourced"

# Start Foxglove in background
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
echo "✓ Foxglove started"

sleep 2




echo "=== Starting Gazebo with args: $@ ==="
echo "PWD: $(pwd)"
echo "World file exists: $(ls -la /workspace/worlds/empty.sdf 2>&1)"

# Start Gazebo server in background
gz sim -s -r -v 4 "$@/world.sdf" &
GAZEBO_PID=$!
echo "✓ Gazebo started with PID $GAZEBO_PID"

sleep 2
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /drone/camera@sensor_msgs/msg/Image[gz.msgs.Image \
  /drone/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /drone/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
  /drone/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat \
  /drone/air_pressure@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure \
  /drone/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model \
  /world/harmonic_world/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V
echo "=== Starting ros_gz_bridge ==="

BRIDGE_PID=$!
echo "✓ ros_gz_bridge started with PID $BRIDGE_PID"

# Keep container alive while both are running
wait $GAZEBO_PID