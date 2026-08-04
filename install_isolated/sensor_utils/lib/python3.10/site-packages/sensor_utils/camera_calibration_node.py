from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class CameraCalibrationNode(Node):
    """Preview stereo cameras and save calibration image pairs with the s key."""

    def __init__(self):
        super().__init__('camera_calibration_node')
        self.declare_parameter('high_image_topic', '/camera/high/image_raw')
        self.declare_parameter('low_image_topic', '/camera/low/image_raw')
        self.declare_parameter('output_dir', 'calibration/stereo')
        self.declare_parameter('window_name', 'camera_calibration')

        self.output_dir = Path(self.get_parameter('output_dir').value).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.window_name = self.get_parameter('window_name').value
        self.latest = {'high': None, 'low': None}
        self.capture_index = self.next_capture_index()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Image,
            self.get_parameter('high_image_topic').value,
            lambda msg: self.image_callback('high', msg),
            qos,
        )
        self.create_subscription(
            Image,
            self.get_parameter('low_image_topic').value,
            lambda msg: self.image_callback('low', msg),
            qos,
        )
        self.timer = self.create_timer(0.03, self.draw)

    def next_capture_index(self):
        existing = sorted(self.output_dir.glob('high-*.png'))
        if not existing:
            return 0
        return int(existing[-1].stem.split('-')[-1]) + 1

    def image_callback(self, side, msg):
        frame = self.to_bgr(msg)
        if frame is not None:
            self.latest[side] = frame

    def to_bgr(self, msg):
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding in ('yuv422_yuy2', 'yuyv', 'yuyv422'):
            return cv2.cvtColor(
                data.reshape((msg.height, msg.width, 2)),
                cv2.COLOR_YUV2BGR_YUY2,
            )
        if msg.encoding in ('bgr8', '8UC3'):
            return data.reshape((msg.height, msg.width, 3))
        if msg.encoding == 'rgb8':
            return cv2.cvtColor(
                data.reshape((msg.height, msg.width, 3)),
                cv2.COLOR_RGB2BGR,
            )
        return None

    def draw(self):
        frames = []
        for side in ('high', 'low'):
            frame = self.latest[side]
            if frame is None:
                frame = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.putText(
                    frame,
                    f'waiting for {side}',
                    (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
            frames.append(frame)

        preview = np.hstack(frames)
        cv2.putText(
            preview,
            'press s to save stereo pair, q to close window',
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow(self.window_name, preview)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            self.save_pair()
        elif key == ord('q'):
            cv2.destroyWindow(self.window_name)

    def save_pair(self):
        if self.latest['high'] is None or self.latest['low'] is None:
            self.get_logger().warn('Both high and low frames are required.')
            return
        high_path = self.output_dir / f'high-{self.capture_index:04d}.png'
        low_path = self.output_dir / f'low-{self.capture_index:04d}.png'
        cv2.imwrite(str(high_path), self.latest['high'])
        cv2.imwrite(str(low_path), self.latest['low'])
        self.get_logger().info(f'Saved {high_path} and {low_path}')
        self.capture_index += 1

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
