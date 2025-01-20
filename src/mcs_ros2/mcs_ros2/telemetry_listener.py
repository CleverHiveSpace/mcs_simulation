import math
import rclpy
import socketio
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, PointCloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rosidl_runtime_py import message_to_ordereddict

class TelemetrySubscriber(Node):

    def __init__(self):
        super().__init__('telemetry_subscriber')
        self.sio = socketio.Client()
        try:
            self.sio.connect('https://localhost:1234')
            self.get_logger().info("Connected to socket.io server.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to socket.io server: {e}")

        imu_sub = Subscriber(self, Imu, '/imu_broadcaster/imu')
        pcl_sub = Subscriber(self, PointCloud2, '/rosbot/laser/point_cloud')
        joint_sub = Subscriber(self, JointState, '/joint_states')

        self.ts = ApproximateTimeSynchronizer([imu_sub, pcl_sub, joint_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synced_callback)

    def synced_callback(self, imu_msg, pcl_msg, joint_msg):

        imu_data = message_to_ordereddict(imu_msg)
        pcl_data  = message_to_ordereddict(pcl_msg)

        joint_data = {
            'timestamp': f"{joint_msg.header.stamp.sec}.{joint_msg.header.stamp.nanosec}",
            'name': joint_msg.name,
            'position': list(joint_msg.position),
            'velocity': list(joint_msg.velocity),
            'effort': self.filter_nan_values(list(joint_msg.effort))
        }

        synchronized_data = {
            'imu': imu_data,
            'point_cloud': pcl_data,
            'joint_states': joint_data  
        }
        
        current_ros_time = self.get_clock().now().to_msg()
        self.get_logger().info(f"Sending synchronized data at ROS time {current_ros_time.sec}.{current_ros_time.nanosec}")

        if self.sio.connected:
            try:
                self.sio.emit('telemetry_data', synchronized_data)
                self.get_logger().debug("Synchronized data emitted successfully.")
            except Exception as e:
                self.get_logger().error(f"Failed to emit data via socket.io: {e}")
        else:
            self.get_logger().warn("Not connected to socket.io server. Data not sent.")
    
    def filter_nan_values(self, values):
        [e if not math.isnan(e) else None for e in values]

    def shutdown(self):
       self.sio.disconnect()


def main(args=None):
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
