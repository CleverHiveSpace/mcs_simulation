import rclpy
import socketio
from rclpy.node import Node

from sensor_msgs.msg import Imu


class ImuSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.sio = socketio.Client()
        self.sio.connect('http://localhost:5000') #TODO: change to mcs / move to env
        self.subscription = self.create_subscription(
            Imu,
            '/imu_broadcaster/imu',
            self.listener_callback,
            100)
        self.subscription 
        self.latest_imu_msg = None
        self.timer = self.create_timer(1.0, self.timer_callback)

    def listener_callback(self, msg):
        self.latest_imu_msg = msg
        self.get_logger().info(f"Received new IMU message at {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")

    def timer_callback(self):
        if self.latest_imu_msg is None:
            return
        msg = self.latest_imu_msg
        self.get_logger().info(f"Received IMU message at time {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        self.get_logger().info(f"Orientation: x={msg.orientation.x}, y={msg.orientation.y}, z={msg.orientation.z}, w={msg.orientation.w}")
        self.get_logger().info(f"Angular Velocity: x={msg.angular_velocity.x}, y={msg.angular_velocity.y}, z={msg.angular_velocity.z}")
        self.get_logger().info(f"Linear Acceleration: x={msg.linear_acceleration.x}, y={msg.linear_acceleration.y}, z={msg.linear_acceleration.z}")
        imu_data = {
            'timestamp': f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}",
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
        }
        if self.sio.connected:
            self.sio.emit('imu_data', imu_data)
        else:
            self.get_logger().warn("Not connected to socket io")

    def shutdown(self):
       self.sio.disconnect()


def main(args=None):
    rclpy.init(args=args)

    imuSubscriber = ImuSubscriber()

    try:
        rclpy.spin(imuSubscriber)
    except KeyboardInterrupt:
        pass
    finally:
        imuSubscriber.shutdown()
        imuSubscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
