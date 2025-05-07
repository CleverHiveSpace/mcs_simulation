import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np
import threading
import time


class ImageDisplayNode(Node):
    def __init__(self):
        super().__init__('image_display_node')

        self.topic_names = [
            '/navcam_front/image_raw/image_color',
            '/navcam_back/image_raw/image_color',
            '/scicam_front/image_raw/image_color',
            '/mastcam/image_raw/image_color'
        ]

        self.bridge = CvBridge()
        self.frames = [None] * len(self.topic_names)
        self.lock = threading.Lock()

        for idx, topic in enumerate(self.topic_names):
            self.create_subscription(
                Image,
                topic,
                self._make_callback(idx),
                10
            )

    def _make_callback(self, index):
        def callback(msg):
            try:
                # Immediate deep copy of converted image
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8').copy()
                with self.lock:
                    self.frames[index] = cv_image
            except Exception as e:
                self.get_logger().error(f"Error processing image from {self.topic_names[index]}: {e}")
        return callback

    def get_frames(self):
        with self.lock:
            return [frame.copy() if frame is not None else None for frame in self.frames]


def display_loop(node: ImageDisplayNode):
    while rclpy.ok():
        frames = node.get_frames()
        canvas = []

        for i in range(0, len(frames), 2):
            row_imgs = []
            for j in range(2):
                idx = i + j
                if idx < len(frames) and frames[idx] is not None:
                    img = cv2.resize(frames[idx], (320, 240))
                else:
                    img = np.zeros((240, 320, 3), dtype=np.uint8)
                row_imgs.append(img)
            row = np.hstack(row_imgs)
            canvas.append(row)

        grid = np.vstack(canvas)
        cv2.imshow("Camera Feeds", grid)

        key = cv2.waitKey(1)
        if key == 27:  # ESC key
            break


def main():
    rclpy.init()
    node = ImageDisplayNode()

    thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    thread.start()

    try:
        display_loop(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
