import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class CameraViewerNode(Node):
    """Show one or two camera image topics with OpenCV windows."""

    def __init__(self):
        super().__init__('camera_viewer_node')
        self.declare_parameter('high_image_topic', '/camera/high/image_raw')
        self.declare_parameter('low_image_topic', '/camera/low/image_raw')
        self.declare_parameter('deprecated_left_image_topic', '/camera/left/image_raw')
        self.declare_parameter('deprecated_right_image_topic', '/camera/right/image_raw')
        self.declare_parameter('window_prefix', 'camera')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.window_prefix = self.get_parameter('window_prefix').value
        self.create_subscription(
            Image,
            self.get_parameter('high_image_topic').value,
            lambda msg: self.show_image('high', msg),
            qos,
        )
        self.create_subscription(
            Image,
            self.get_parameter('low_image_topic').value,
            lambda msg: self.show_image('low', msg),
            qos,
        )
        self.create_subscription(
            Image,
            self.get_parameter('deprecated_left_image_topic').value,
            lambda msg: self.show_image('deprecated_left', msg),
            qos,
        )
        self.create_subscription(
            Image,
            self.get_parameter('deprecated_right_image_topic').value,
            lambda msg: self.show_image('deprecated_right', msg),
            qos,
        )
        self.timer = self.create_timer(0.03, lambda: cv2.waitKey(1))

    def show_image(self, side, msg):
        frame = self.to_bgr(msg)
        if frame is None:
            return
        # Keep the received image buffer untouched and draw a red guide at the
        # exact horizontal midpoint of every camera view.
        frame = frame.copy()
        height, width = frame.shape[:2]
        center_x = width // 2
        cv2.line(
            frame,
            (center_x, 0),
            (center_x, height - 1),
            (0, 0, 255),
            2,
        )
        cv2.imshow(f'{self.window_prefix}_{side}', frame)

    def to_bgr(self, msg):
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding in ('yuv422_yuy2', 'yuyv', 'yuyv422'):
            yuyv = data.reshape((msg.height, msg.width, 2))
            return cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)
        if msg.encoding in ('bgr8', '8UC3'):
            return data.reshape((msg.height, msg.width, 3))
        if msg.encoding == 'rgb8':
            rgb = data.reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if msg.encoding in ('mono8', '8UC1'):
            mono = data.reshape((msg.height, msg.width))
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

        self.get_logger().warn(
            f'Unsupported camera encoding: {msg.encoding}',
            throttle_duration_sec=5.0,
        )
        return None

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
