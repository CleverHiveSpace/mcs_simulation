from __future__ import annotations

import time
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional

import cv2
import rclpy
import os
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

VIDEO_DIR = Path(os.getenv("VIDEO_DIR", "/tmp/edge_rec"))
VIDEO_DIR.mkdir(parents=True, exist_ok=True)

FPS: float = 30.0
RETENTION_MINUTES: int = 30
MAX_DISK_GB: float = 1.0

FOURCC: int = cv2.VideoWriter_fourcc(*"mp4v")

CAMERA_TOPIC: str = "/navcam_front/image_raw/image_color"
CAMERA_NAME: str = "navcam_front"


class NavcamRecorder(Node):
    """Subscribe to the nav-cam image topic and save 1-minute MP4 segments.

    A new file starts precisely on each UTC minute boundary (…10:30, …10:31, …10:32…).
    The filename includes only the date, hour and minute so you can immediately see
    which minute a file covers.
    """

    def __init__(self) -> None:
        super().__init__(CAMERA_NAME)

        self.bridge = CvBridge()
        self.writer: Optional[cv2.VideoWriter] = None
        self.current_minute: Optional[datetime] = None  

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, CAMERA_TOPIC, self._callback, qos)
        self.create_timer(60.0, self._cleanup_files)

    @staticmethod
    def _new_filename(ts: datetime) -> str:
        """Build a filename like navcam_front_20250617_1030.mp4.

        The *ts* argument must already be truncated to the minute (second = 0).
        """
        return str(VIDEO_DIR / f"{CAMERA_NAME}_{ts.strftime('%Y%m%d_%H%M')}.mp4")

    def _open_writer(self, width: int, height: int, ts: datetime) -> cv2.VideoWriter:
        path = self._new_filename(ts)
        writer = cv2.VideoWriter(path, FOURCC, FPS, (width, height))
        if not writer.isOpened():
            raise RuntimeError(f"Failed to open VideoWriter for {path}")
        self.get_logger().info(f"Started new segment {path}")
        return writer

    def _callback(self, msg: Image) -> None:
        """Receive a frame and write it to the current minute-aligned segment."""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]

        now = datetime.now(timezone.utc)
        minute_mark = now.replace(second=0, microsecond=0)

        if self.writer is None or minute_mark != self.current_minute:
            if self.writer is not None:
                self.writer.release()
            self.writer = self._open_writer(w, h, minute_mark)
            self.current_minute = minute_mark

        assert self.writer is not None
        self.writer.write(frame)

    def _cleanup_files(self) -> None:
        files: List[Path] = sorted(VIDEO_DIR.glob("*.mp4"), key=lambda p: p.stat().st_mtime)
        cutoff = time.time() - RETENTION_MINUTES * 60

        for f in files:
            if f.stat().st_mtime < cutoff:
                try:
                    f.unlink()
                    self.get_logger().info(f"Deleted old file (age): {f}")
                except Exception as exc:
                    self.get_logger().warn(f"Failed to delete {f}: {exc}")

        files = sorted(VIDEO_DIR.glob("*.mp4"), key=lambda p: p.stat().st_mtime)
        while files and sum(p.stat().st_size for p in files) / 1e9 > MAX_DISK_GB:
            oldest = files.pop(0)
            try:
                oldest.unlink()
                self.get_logger().info(f"Deleted old file (size): {oldest}")
            except Exception as exc:
                self.get_logger().warn(f"Failed to delete {oldest}: {exc}")

    def shutdown(self) -> None:
        if self.writer is not None:
            self.writer.release()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = NavcamRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
