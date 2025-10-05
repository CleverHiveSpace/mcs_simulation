#!/usr/bin/env python3
import time
import math
import requests
import socketio
import argparse
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# Configuration
BACKEND_URL = "https://api.rovers.website"


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def login(username, password):
    """Login to get access token."""
    url = f"{BACKEND_URL}/login"
    payload = {"username": username, "password": password}

    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()
        data = response.json()
        token = data.get("accessToken")
        print(f"✓ Successfully logged in as {username}")
        return token
    except requests.exceptions.RequestException as e:
        print(f"✗ Login failed: {e}")
        return None


class MCSBridgeNode(Node):
    """ROS 2 Node that bridges odometry and IMU data to MCS backend."""

    def __init__(self, rover_id, sio, namespace):
        super().__init__("mcs_bridge_node")
        self.rover_id = rover_id
        self.sio = sio
        self.namespace = namespace

        # Subscribe to odometry topic
        self.odom_subscription = self.create_subscription(
            Odometry, "/odometry/filtered", self.odometry_callback, 10
        )

        # Subscribe to IMU topic
        self.imu_subscription = self.create_subscription(
            Imu, "/imu_broadcaster/imu", self.imu_callback, 10
        )

        self.get_logger().info(f"MCS Bridge Node initialized for rover: {rover_id}")
        self.get_logger().info("Subscribed to /odometry/filtered")
        self.get_logger().info("Subscribed to /imu_broadcaster/imu")

    def odometry_callback(self, msg):
        """Callback for odometry messages."""
        try:
            # Extract position
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z

            # Extract orientation quaternion and convert to yaw angle
            orientation = msg.pose.pose.orientation
            yaw = quaternion_to_yaw(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
            angle_deg = math.degrees(yaw) % 360

            # Create position data payload
            position_data = {
                "roverId": self.rover_id,
                "ts_ns": int(time.time() * 1_000_000_000),  # nanoseconds
                "x": x,
                "y": y,
                "z": z,
                "angle": angle_deg,
            }

            # Send to backend
            if self.sio.connected:
                self.sio.emit("position", position_data, namespace=self.namespace)
                self.get_logger().info(
                    f"→ Position: x={x:.2f}, y={y:.2f}, z={z:.2f}, angle={angle_deg:.1f}°"
                )
                self.get_logger().debug(f"Position data sent: {position_data}")
            else:
                self.get_logger().warn(
                    "WebSocket not connected, cannot send position data"
                )
        except Exception as e:
            self.get_logger().error(f"Error in odometry callback: {e}")

    def imu_callback(self, msg):
        """Callback for IMU messages."""
        try:
            # Extract linear acceleration
            accel_x = msg.linear_acceleration.x
            accel_y = msg.linear_acceleration.y
            accel_z = msg.linear_acceleration.z

            # Send each axis as separate sensor data
            sensors = [
                {
                    "roverId": self.rover_id,
                    "ts_ns": int(time.time() * 1_000_000_000),  # nanoseconds
                    "name": "imu_accel_x",
                    "value": accel_x,
                    "unit": "m/s²",
                },
                {
                    "roverId": self.rover_id,
                    "ts_ns": int(time.time() * 1_000_000_000),  # nanoseconds
                    "name": "imu_accel_y",
                    "value": accel_y,
                    "unit": "m/s²",
                },
                {
                    "roverId": self.rover_id,
                    "ts_ns": int(time.time() * 1_000_000_000),  # nanoseconds
                    "name": "imu_accel_z",
                    "value": accel_z,
                    "unit": "m/s²",
                },
            ]

            # Send to backend
            if self.sio.connected:
                for sensor_data in sensors:
                    self.sio.emit("sensor", sensor_data, namespace=self.namespace)
                    self.get_logger().debug(f"Sensor data sent: {sensor_data}")

                self.get_logger().info(
                    f"→ IMU: x={accel_x:.2f}, y={accel_y:.2f}, z={accel_z:.2f} m/s²"
                )
            else:
                self.get_logger().warn(
                    "WebSocket not connected, cannot send sensor data"
                )
        except Exception as e:
            self.get_logger().error(f"Error in IMU callback: {e}")


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="ROS2 MCS Bridge - Send telemetry data to MCS backend",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --username test1 --password test1 --rover-id r2d2
  %(prog)s -u admin -p secret123 -r rover01

This ROS 2 node subscribes to /odometry/filtered and /imu_broadcaster/imu topics
and forwards the data to the MCS backend via WebSocket.
        """,
    )

    parser.add_argument(
        "--username", "-u", required=True, help="Username for authentication (required)"
    )

    parser.add_argument(
        "--password", "-p", required=True, help="Password for authentication (required)"
    )

    parser.add_argument(
        "--rover-id", "-r", required=True, help="Rover ID for telemetry data (required)"
    )

    return parser.parse_args()


def main():
    """Main function to initialize ROS 2 node and connect to MCS backend."""
    # Parse command line arguments
    args = parse_args()

    print(f"Starting ROS2 MCS Bridge for rover: {args.rover_id}")
    print(f"Backend URL: {BACKEND_URL}")
    print()

    # Login first
    print("Logging in...")
    token = login(args.username, args.password)

    if not token:
        print("Failed to login. Exiting.")
        return

    # Create Socket.IO client
    import logging

    logging.basicConfig(level=logging.DEBUG)
    sio = socketio.Client(
        logger=False,
        engineio_logger=False,
        ssl_verify=False,  # disable SSL verification for to make requests come through from http origin
    )

    @sio.event
    def connect():
        print("✓ Connected to WebSocket namespace /ws/robot")

    @sio.event
    def disconnect():
        print("✓ Disconnected from WebSocket")

    @sio.event
    def connect_error(data):
        print("✗ Connection error:")
        print(f"   Type: {type(data)}")
        print(f"   Data: {data}")
        if hasattr(data, "__dict__"):
            print(f"   Details: {data.__dict__}")

    # Connect to the telemetry namespace
    namespace = "/ws/robot"
    print(f"Connecting to {BACKEND_URL}{namespace}...")
    print(f"Using token: {token[:20]}...")

    try:
        sio.connect(
            BACKEND_URL,
            namespaces=[namespace],
            transports=["websocket"],
            wait_timeout=10,
        )

        # Wait for connection to be fully established
        if not sio.connected:
            print("Waiting for connection...")
            time.sleep(2)

        if not sio.connected:
            print("✗ Failed to connect to WebSocket. Exiting.")
            return

        print("✓ WebSocket connected successfully")
        print()

        # Initialize ROS 2
        rclpy.init()

        # Create the MCS Bridge node
        bridge_node = MCSBridgeNode(args.rover_id, sio, namespace)

        print("ROS 2 node started. Listening to topics...")
        print("  - /odometry/filtered")
        print("  - /imu_broadcaster/imu")
        print("Press Ctrl+C to stop\n")

        # Spin the node
        try:
            rclpy.spin(bridge_node)
        except KeyboardInterrupt:
            print("\n\nStopping ROS 2 MCS Bridge...")
        finally:
            # Cleanup
            bridge_node.destroy_node()
            rclpy.shutdown()

    except socketio.exceptions.ConnectionError as e:
        print("✗ Socket.IO Connection Error:")
        print(f"   Message: {e}")
        print(f"   Type: {type(e)}")
        import traceback

        traceback.print_exc()
    except Exception as e:
        print("✗ Unexpected Error:")
        print(f"   Message: {e}")
        print(f"   Type: {type(e)}")
        import traceback

        traceback.print_exc()
    finally:
        if sio.connected:
            sio.disconnect()
        print("✓ Disconnected")


if __name__ == "__main__":
    main()
