import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import time


class ImageFPSMonitor(Node):
    def __init__(self):
        super().__init__('image_fps_monitor')

        self.topic_names = [
            '/navcam_front/image_raw/image_color',
            '/navcam_back/image_raw/image_color',
            '/scicam_front/image_raw/image_color',
            '/mastcam/image_raw/image_color'
        ]

        self.last_print_time = time.time()
        self.frame_counts = [0] * len(self.topic_names)
        self.fps = [0.0] * len(self.topic_names)

        self.image_subs = []
        for idx, topic in enumerate(self.topic_names):
            sub = self.create_subscription(
                Image,
                topic,
                self._make_callback(idx),
                10
            )
            self.image_subs.append(sub)

        # Timer to print FPS every 1 second
        self.timer = self.create_timer(1.0, self.print_fps)

    def _make_callback(self, index):
        def callback(msg):
            self.frame_counts[index] += 1
        return callback

    def print_fps(self):
        current_time = time.time()
        elapsed = current_time - self.last_print_time
        self.last_print_time = current_time

        for i in range(len(self.topic_names)):
            self.fps[i] = self.frame_counts[i] / elapsed
            self.frame_counts[i] = 0

        self.get_logger().info('--- Frame Rates (FPS) ---')
        for i, fps in enumerate(self.fps):
            timestamp = time.strftime('%H:%M:%S')
            self.get_logger().info(f'[{timestamp}] {self.topic_names[i]}: {fps:.2f} FPS')


def main(args=None):
    rclpy.init(args=args)
    node = ImageFPSMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
