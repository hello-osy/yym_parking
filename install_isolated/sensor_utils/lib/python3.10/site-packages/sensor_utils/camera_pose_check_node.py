import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class CameraPoseCheckNode(Node):
    """Show camera feeds with center guides and optional chessboard detection."""

    def __init__(self):
        super().__init__('camera_pose_check_node')
        self.declare_parameter('high_image_topic', '/camera/high/image_raw')
        self.declare_parameter('low_image_topic', '/camera/low/image_raw')
        self.declare_parameter('chessboard_cols', 8)
        self.declare_parameter('chessboard_rows', 6)
        self.declare_parameter('window_name', 'camera_pose_check')

        self.pattern_size = (
            int(self.get_parameter('chessboard_cols').value),
            int(self.get_parameter('chessboard_rows').value),
        )
        self.window_name = self.get_parameter('window_name').value
        self.latest = {'high': None, 'low': None}

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
            else:
                frame = frame.copy()
                self.draw_guides(frame)
                self.draw_chessboard(frame)
            frames.append(frame)

        cv2.imshow(self.window_name, np.hstack(frames))
        cv2.waitKey(1)

    def draw_guides(self, frame):
        height, width = frame.shape[:2]
        cv2.line(frame, (width // 2, 0), (width // 2, height), (0, 255, 0), 1)
        cv2.line(frame, (0, height // 2), (width, height // 2), (0, 255, 0), 1)
        cv2.rectangle(frame, (10, 10), (width - 10, height - 10), (0, 120, 0), 1)

    def draw_chessboard(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, self.pattern_size)
        if found:
            cv2.drawChessboardCorners(frame, self.pattern_size, corners, found)
            cv2.putText(
                frame,
                'chessboard detected',
                (20, frame.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPoseCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
