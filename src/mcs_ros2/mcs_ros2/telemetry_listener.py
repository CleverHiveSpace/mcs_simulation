import math
import rclpy
import socketio
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, PointCloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rosidl_runtime_py import message_to_ordereddict
from typing import Optional, Any, Dict, List
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped


class TelemetrySubscriber(Node):

    def __init__(self) -> None:
        super().__init__('telemetry_subscriber')
        self.sio: socketio.Client = socketio.Client()
        try:
            self.sio.connect('https://localhost:1234')
            self.get_logger().info("Connected to socket.io server.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to socket.io server: {e}")

        imu_sub: Subscriber = Subscriber(self, Imu, '/imu_broadcaster/imu')
        odom_sub: Subscriber = Subscriber(self, Odometry, '/odometry/filtered')
        north_vector_sub: Subscriber = Subscriber(self, Vector3Stamped, '/rosbot/imu_compass/north_vector')

        self.ts: ApproximateTimeSynchronizer = ApproximateTimeSynchronizer(
            [imu_sub, odom_sub, north_vector_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.synced_callback)

    def synced_callback(self, 
                        imu_msg: Imu, 
                        odom_msg: Odometry, 
                        north_vector_msg: Vector3Stamped) -> None:
        imu_data: Dict[str, Any] = message_to_ordereddict(imu_msg)
        
        north_vector_data: Dict[str, Any] = message_to_ordereddict(north_vector_msg)

        odom_data: Dict[str, Any] = {
            'timestamp': f"{odom_msg.header.stamp.sec}.{odom_msg.header.stamp.nanosec}",
            'position': {
                'longitude': odom_msg.pose.pose.position.x,
                'latitude': odom_msg.pose.pose.position.y,
                'altitude': odom_msg.pose.pose.position.z
            },
            'orientation': {
                'roll': odom_msg.pose.pose.orientation.x,
                'pitch': odom_msg.pose.pose.orientation.y,
                'yaw': odom_msg.pose.pose.orientation.z
            }
        }

        synchronized_data: Dict[str, Any] = {
            'imu': imu_data,
            'odom': odom_data,
            'north_vector': north_vector_data
        }
        
        current_ros_time = self.get_clock().now().to_msg()
        self.get_logger().info(
            f"Sending synchronized data at ROS time {current_ros_time.sec}.{current_ros_time.nanosec}"
        )

        if self.sio.connected:
            try:
                self.sio.emit('telemetry_data', synchronized_data)
                self.get_logger().debug("Synchronized data emitted successfully.")
            except Exception as e:
                self.get_logger().error(f"Failed to emit data via socket.io: {e}")
        else:
            self.get_logger().warn("Not connected to socket.io server. Data not sent.")
    
    def filter_nan_values(self, values: List[float]) -> List[Optional[float]]:
        return [e if not math.isnan(e) else None for e in values]

    def shutdown(self) -> None:
        self.sio.disconnect()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)

    telemetrySubscriber = TelemetrySubscriber()

    try:
        rclpy.spin(telemetrySubscriber)
    except KeyboardInterrupt:
        pass
    finally:
        telemetrySubscriber.shutdown()
        telemetrySubscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
