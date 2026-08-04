import threading
import time
from glob import glob
from pathlib import Path

import cv2
import rclpy
import yaml
from rclpy.node import Node
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def load_camera_info(path, width, height):
    msg = CameraInfo()
    msg.width = width
    msg.height = height

    if not path:
        return msg

    info_path = Path(path).expanduser()
    if not info_path.exists():
        return msg

    data = yaml.safe_load(info_path.read_text()) or {}
    msg.width = int(data.get('image_width', width))
    msg.height = int(data.get('image_height', height))
    msg.distortion_model = data.get('distortion_model', 'plumb_bob')
    msg.k = [float(x) for x in data.get('camera_matrix', {}).get('data', [0.0] * 9)]
    msg.d = [
        float(x)
        for x in data.get('distortion_coefficients', {}).get('data', [])
    ]
    msg.r = [
        float(x)
        for x in data.get('rectification_matrix', {}).get('data', [0.0] * 9)
    ]
    msg.p = [
        float(x)
        for x in data.get('projection_matrix', {}).get('data', [0.0] * 12)
    ]
    return msg


class CameraPublisher:
    def __init__(self, node, side, device=None):
        self.node = node
        self.side = side
        self.device = device or node.get_parameter(f'{side}.device').value
        self.frame_id = node.get_parameter(f'{side}.frame_id').value
        self.image_topic = node.get_parameter(f'{side}.image_topic').value
        self.info_topic = node.get_parameter(f'{side}.camera_info_topic').value
        self.camera_info_path = node.get_parameter(f'{side}.camera_info').value
        self.width = int(node.get_parameter('width').value)
        self.height = int(node.get_parameter('height').value)
        self.fps = float(node.get_parameter('fps').value)
        self.pixel_format = node.get_parameter('pixel_format').value
        self.required_name_substring = node.get_parameter(
            'required_name_substring'
        ).value

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_pub = node.create_publisher(Image, self.image_topic, qos)
        self.info_pub = node.create_publisher(CameraInfo, self.info_topic, qos)
        self.cap = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.last_open_attempt = 0.0
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.close()

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def try_open(self):
        now = time.monotonic()
        if now - self.last_open_attempt < 1.0:
            return False
        self.last_open_attempt = now

        if not self.node.device_name_matches(self.device, self.required_name_substring):
            self.node.get_logger().warn(
                f'[{self.side}] waiting for C920 camera on {self.device}',
                throttle_duration_sec=5.0,
            )
            return False

        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.node.get_logger().warn(
                f'[{self.side}] waiting for camera device {self.device}',
                throttle_duration_sec=5.0,
            )
            return False

        fourcc = cv2.VideoWriter_fourcc(*self.pixel_format)
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

        self.cap = cap
        self.actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.actual_fps = cap.get(cv2.CAP_PROP_FPS)
        self.image_step = self.actual_width * 2
        self.camera_info = load_camera_info(
            self.camera_info_path,
            self.actual_width,
            self.actual_height,
        )
        self.node.get_logger().info(
            f'[{self.side}] opened {self.device}: {self.actual_width}x'
            f'{self.actual_height}@{self.actual_fps:.2f}, publishing '
            f'{self.image_topic} and {self.info_topic}'
        )
        return True

    def capture_loop(self):
        while rclpy.ok() and not self.stop_event.is_set():
            if self.cap is None and not self.try_open():
                time.sleep(0.1)
                continue

            ok, frame = self.cap.read()
            if not ok or frame is None:
                self.node.get_logger().warn(
                    f'[{self.side}] failed to read frame, reopening camera',
                    throttle_duration_sec=2.0,
                )
                self.close()
                continue

            try:
                self.publish_frame(frame)
            except _rclpy.RCLError:
                # Ctrl+C로 ROS context가 먼저 종료된 경우 publish 경쟁을 조용히 끝낸다.
                if self.stop_event.is_set() or not rclpy.ok():
                    break
                raise

    def publish_frame(self, frame):
        stamp = self.node.get_clock().now().to_msg()
        data = frame.tobytes()

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self.frame_id
        image.height = self.actual_height
        image.width = self.actual_width
        image.encoding = 'yuv422_yuy2'
        image.is_bigendian = False
        image.step = self.image_step
        image.data = data

        info = CameraInfo()
        info.header = image.header
        info.width = self.camera_info.width or image.width
        info.height = self.camera_info.height or image.height
        info.distortion_model = self.camera_info.distortion_model
        info.d = self.camera_info.d
        info.k = self.camera_info.k
        info.r = self.camera_info.r
        info.p = self.camera_info.p

        self.image_pub.publish(image)
        self.info_pub.publish(info)


class CameraNode(Node):
    """Publish high/low USB camera frames and CameraInfo messages."""

    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 360)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('pixel_format', 'YUYV')
        self.declare_parameter('auto_fallback_devices', True)
        self.declare_parameter('required_name_substring', 'C920')
        self.declare_parameter('enable_high', True)
        self.declare_parameter('enable_low', True)

        for side, device in (('high', '/dev/video4'), ('low', '/dev/video6')):
            self.declare_parameter(f'{side}.device', device)
            self.declare_parameter(f'{side}.frame_id', f'camera_{side}')
            self.declare_parameter(f'{side}.image_topic', f'/camera/{side}/image_raw')
            self.declare_parameter(
                f'{side}.camera_info_topic',
                f'/camera/{side}/camera_info',
            )
            self.declare_parameter(
                f'{side}.camera_info',
                f'~/.ros/camera_info/c920_{side}.yaml',
            )

        used_devices = set()
        self.camera_publishers = []
        enabled_sides = []
        if bool(self.get_parameter('enable_high').value):
            enabled_sides.append('high')
        if bool(self.get_parameter('enable_low').value):
            enabled_sides.append('low')
        for side in enabled_sides:
            preferred = self.get_parameter(f'{side}.device').value
            device = self.resolve_device(side, preferred, used_devices)
            if device:
                used_devices.add(self.device_key(device))
            self.camera_publishers.append(CameraPublisher(self, side, device))

    def resolve_device(self, side, preferred, used_devices):
        if preferred != 'auto' and self.is_capture_device(preferred):
            return preferred
        if not bool(self.get_parameter('auto_fallback_devices').value):
            return preferred

        for device in self.camera_candidates():
            if self.device_key(device) in used_devices:
                continue
            if self.is_capture_device(device):
                self.get_logger().warn(
                    f'[{side}] {preferred} not found, using fallback {device}'
                )
                return device

        return preferred

    def camera_candidates(self):
        candidates = []
        candidates.extend(sorted(glob('/dev/v4l/by-path/*')))
        candidates.extend(sorted(glob('/dev/v4l/by-id/*')))
        candidates.extend(sorted(glob('/dev/video*')))

        unique = []
        seen = set()
        for device in candidates:
            key = self.device_key(device)
            if key in seen:
                continue
            seen.add(key)
            unique.append(device)
        return unique

    def is_capture_device(self, device):
        required = self.get_parameter('required_name_substring').value
        if not self.device_name_matches(device, required):
            return False

        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        try:
            if not cap.isOpened():
                return False
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            return width > 0 and height > 0
        finally:
            cap.release()

    def device_name_matches(self, device, required):
        if not required:
            return True

        device_path = Path(device)
        if not device_path.exists():
            return False

        try:
            video_name = device_path.resolve().name
            name_path = Path('/sys/class/video4linux') / video_name / 'name'
            device_name = name_path.read_text().strip()
        except OSError:
            return False

        return required.lower() in device_name.lower()

    def device_key(self, device):
        try:
            return str(Path(device).resolve())
        except OSError:
            return str(device)

    def destroy_node(self):
        for publisher in getattr(self, 'camera_publishers', []):
            publisher.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
