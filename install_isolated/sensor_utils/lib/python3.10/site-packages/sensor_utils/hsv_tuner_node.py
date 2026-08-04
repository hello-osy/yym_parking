"""hsv_tuner_node.

카메라 토픽(bag 재생이든 실제 카메라든 상관없음)을 구독해서 트랙바로
HSV 최소/최대값을 조절하며 마스크 결과를 실시간으로 보여준다.
lane_offset의 흰색/초록색 임계값을 눈으로 보면서 정확히 잡을 때 사용.

사용 예:
    ros2 run sensor_utils hsv_tuner_node --ros-args -p image_topic:=/camera/high/image_raw
    ros2 run sensor_utils hsv_tuner_node --ros-args -p preset:=green
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

# ============================================================================
# 파라미터 기본값 - 튜닝은 대부분 여기서만 하면 된다.
# (전부 ROS 파라미터로도 선언되므로 --ros-args -p 로 실행 중 덮어쓰기도 가능)
# ============================================================================
IMAGE_TOPIC = '/camera/high/image_raw'
PRESET = 'white'

WIN_CONTROLS = 'hsv_tuner_controls'
WIN_ORIGINAL = 'hsv_tuner_original'
WIN_MASK = 'hsv_tuner_mask'
WIN_RESULT = 'hsv_tuner_result'

# lane_offset 에서 이미 쓰고 있는 값들을 시작점으로 제공 (거기서부터 미세 조정)
PRESETS = {
    # H는 흰색 판별에 안 쓰므로 전체 범위로 둠
    'white': {'h': (0, 179), 's': (0, 60), 'v': (140, 255)},
    'green': {'h': (30, 90), 's': (40, 255), 'v': (70, 255)},
    'full': {'h': (0, 179), 's': (0, 255), 'v': (0, 255)},
}


class HsvTunerNode(Node):
    """트랙바로 HSV 범위를 조절하며 원본/마스크/결과 화면을 동시에 보여준다."""

    def __init__(self):
        super().__init__('hsv_tuner_node')

        self.declare_parameter('image_topic', IMAGE_TOPIC)
        self.declare_parameter('preset', PRESET)

        self.image_topic = self.get_parameter('image_topic').value
        preset = PRESETS.get(self.get_parameter('preset').value, PRESETS['white'])

        self.frame = None
        self.setup_windows(preset)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(Image, self.image_topic, self.image_callback, qos)
        self.timer = self.create_timer(1.0 / 30.0, self.tick)

        self.get_logger().info(
            f'Subscribing {self.image_topic}. Adjust trackbars in "{WIN_CONTROLS}" '
            'and watch the mask window.'
        )

    def setup_windows(self, preset):
        cv2.namedWindow(WIN_CONTROLS, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN_CONTROLS, 420, 260)
        cv2.namedWindow(WIN_ORIGINAL, cv2.WINDOW_NORMAL)
        cv2.namedWindow(WIN_MASK, cv2.WINDOW_NORMAL)
        cv2.namedWindow(WIN_RESULT, cv2.WINDOW_NORMAL)

        def nothing(_):
            pass

        h_lo, h_hi = preset['h']
        s_lo, s_hi = preset['s']
        v_lo, v_hi = preset['v']
        cv2.createTrackbar('H min', WIN_CONTROLS, h_lo, 179, nothing)
        cv2.createTrackbar('H max', WIN_CONTROLS, h_hi, 179, nothing)
        cv2.createTrackbar('S min', WIN_CONTROLS, s_lo, 255, nothing)
        cv2.createTrackbar('S max', WIN_CONTROLS, s_hi, 255, nothing)
        cv2.createTrackbar('V min', WIN_CONTROLS, v_lo, 255, nothing)
        cv2.createTrackbar('V max', WIN_CONTROLS, v_hi, 255, nothing)

    def image_callback(self, msg):
        self.frame = self.to_bgr(msg)

    def tick(self):
        if self.frame is None:
            cv2.waitKey(1)
            return

        h_min = cv2.getTrackbarPos('H min', WIN_CONTROLS)
        h_max = cv2.getTrackbarPos('H max', WIN_CONTROLS)
        s_min = cv2.getTrackbarPos('S min', WIN_CONTROLS)
        s_max = cv2.getTrackbarPos('S max', WIN_CONTROLS)
        v_min = cv2.getTrackbarPos('V min', WIN_CONTROLS)
        v_max = cv2.getTrackbarPos('V max', WIN_CONTROLS)

        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))
        result = cv2.bitwise_and(self.frame, self.frame, mask=mask)
        pixel_count = int(np.count_nonzero(mask))
        ratio = pixel_count / mask.size

        overlay = self.frame.copy()
        cv2.putText(
            overlay,
            f'H[{h_min},{h_max}] S[{s_min},{s_max}] V[{v_min},{v_max}] '
            f'px={pixel_count} ratio={ratio:.3f}',
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        cv2.imshow(WIN_ORIGINAL, overlay)
        cv2.imshow(WIN_MASK, mask)
        cv2.imshow(WIN_RESULT, result)
        cv2.waitKey(1)

    # ======================================================================
    # YUYV -> BGR 변환 (sensor_utils/camera_viewer_node.py 와 동일한 방식)
    # ======================================================================
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
            f'Unsupported camera encoding: {msg.encoding}', throttle_duration_sec=5.0
        )
        return None

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HsvTunerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
