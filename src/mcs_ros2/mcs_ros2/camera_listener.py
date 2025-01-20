import base64
import cv2
import rclpy
import socketio
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from typing import Optional, Any

class CameraSubscriber(Node):

    def __init__(self) -> None:
        super().__init__('camera_subscriber')
        self.sio: socketio.Client = socketio.Client()

        self.sio.on('connect', self.on_connect)
        self.sio.on('connect_error', self.on_connect_error)
        self.sio.on('disconnect', self.on_disconnect)
        try:
            self.sio.connect('https://localhost:1234')
            self.get_logger().info("Connected to socket.io server.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to socket.io server: {e}")

        self.bridge: CvBridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/rosbot/camera_rgb/image_color',
            self.listener_callback,
            100)

    def listener_callback(self, msg: Image) -> None:
        self.get_logger().info(f"Received camera frame at {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            _, jpeg_buffer = cv2.imencode('.jpg', cv_image)
            jpeg_base64: str = base64.b64encode(jpeg_buffer).decode('utf-8')
            self.send_image_base64(jpeg_base64)

        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def send_image_base64(self, jpeg_base64: str) -> None:
        if self.sio.connected:
            self.sio.emit("camera", {'data': jpeg_base64 })
        else:
            self.get_logger().warn("Message could not be sent: not connected to Socket.IO.")

    def on_connect(self) -> None:
        self.get_logger().info("Successfully connected to Socket.IO server.")

    def on_connect_error(self) -> None:
        self.get_logger().error("Failed to connect to Socket.IO server.")

    def on_disconnect(self) -> None:
        self.get_logger().warn("Disconnected from Socket.IO server.")

    def shutdown(self) -> None:
       self.sio.disconnect()

def main(args: Optional[Any] = None) -> None:
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
