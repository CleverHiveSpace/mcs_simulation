import base64
import cv2
import rclpy
import socketio
import sys
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        self.sio = socketio.Client()
        self.bridge = CvBridge()
        # self.sio.connect('http://localhost:5000') #TODO: change to mcs / move to env
        self.subscription = self.create_subscription(
            Image,
            '/rosbot/camera_rgb/image_color',
            self.listener_callback,
            100)

    def listener_callback(self, msg):
        self.get_logger().info(f"Received camera frame at {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            _, jpeg_buffer = cv2.imencode('.jpg', cv_image)
            jpeg_base64 = base64.b64encode(jpeg_buffer).decode('utf-8')
            jpeg_filename = "output_image.jpg"
            cv2.imwrite(jpeg_filename, cv_image)
            self.send_image_base64(jpeg_base64)
            sys.exit()

        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def send_image_base64(self, jpeg_base64):
        if self.sio.connected:
            self.sio.emit('camera', {'data': jpeg_base64 })
            self.get_logger().info('Image send as base64')
        else:
            self.get_logger().warn("Not connected to socket io")

    def shutdown(self):
       self.sio.disconnect()
'''
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
'''

def main(args=None):
    rclpy.init(args=args)

    camera_subscriber = CameraSubscriber()

    try:
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        camera_subscriber.shutdown()
        camera_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
