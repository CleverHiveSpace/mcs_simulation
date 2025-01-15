import math
from threading import current_thread
import rclpy
import socketio
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, PointCloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer

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
        imu_data = {
            'timestamp': f"{imu_msg.header.stamp.sec}.{imu_msg.header.stamp.nanosec}",
            'orientation': {
                'x': imu_msg.orientation.x,
                'y': imu_msg.orientation.y,
                'z': imu_msg.orientation.z,
                'w': imu_msg.orientation.w
            },
            'angular_velocity': {
                'x': imu_msg.angular_velocity.x,
                'y': imu_msg.angular_velocity.y,
                'z': imu_msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': imu_msg.linear_acceleration.x,
                'y': imu_msg.linear_acceleration.y,
                'z': imu_msg.linear_acceleration.z
            }
        }

        pcl_data = {
            'timestamp': f"{pcl_msg.header.stamp.sec}.{pcl_msg.header.stamp.nanosec}",
            'frame_id': pcl_msg.header.frame_id,
            'width': pcl_msg.width,
            'height': pcl_msg.height,
            'point_count': pcl_msg.width * pcl_msg.height
        }

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
