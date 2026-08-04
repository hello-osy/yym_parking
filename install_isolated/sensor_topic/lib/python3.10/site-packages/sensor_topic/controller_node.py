from glob import glob
import os
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JS_EVENT_FORMAT = 'IhBB'
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
JS_AXIS_MAX = 32767.0

INPUT_EVENT_FORMAT = 'llHHi'
INPUT_EVENT_SIZE = struct.calcsize(INPUT_EVENT_FORMAT)
EV_SYN = 0x00
EV_KEY = 0x01
EV_ABS = 0x03

DEFAULT_ABS_AXIS_ORDER = [0, 1, 2, 3, 4, 5, 16, 17]
DEFAULT_BUTTON_BASE = 0x120


class ControllerNode(Node):
    """Read joystick or evdev controller input and publish sensor_msgs/Joy."""

    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('device_path', 'auto')
        self.declare_parameter('device_type', 'auto')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('topic_name', '/manual_controller/joy')
        self.declare_parameter('event_axis_max', 32767.0)

        self.device_path = self.get_parameter('device_path').value
        self.device_type = self.get_parameter('device_type').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.event_axis_max = float(self.get_parameter('event_axis_max').value)
        topic_name = self.get_parameter('topic_name').value

        self.publisher = self.create_publisher(Joy, topic_name, 10)
        self.device_fd = None
        self.active_device_path = ''
        self.active_device_type = ''
        self.axes = []
        self.buttons = []
        self.event_axis_map = {
            code: index for index, code in enumerate(DEFAULT_ABS_AXIS_ORDER)
        }
        self.event_button_map = {}
        self.last_error_log_time = 0.0
        self.last_reconnect_attempt_time = 0.0

        timer_period = 1.0 / publish_rate if publish_rate > 0.0 else 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'Publishing controller input from {self.device_path} to {topic_name}'
        )

    def destroy_node(self):
        self.close_device()
        super().destroy_node()

    def timer_callback(self):
        if self.device_fd is None:
            self.try_open_device()
            return

        try:
            if self.active_device_type == 'event':
                self.read_pending_event_events()
            else:
                self.read_pending_js_events()
        except OSError as exc:
            self.log_error_throttled(f'Controller disconnected or unreadable: {exc}')
            self.close_device()
            return

        self.publish_joy()

    def try_open_device(self):
        now = time.monotonic()
        if now - self.last_reconnect_attempt_time < 1.0:
            return

        self.last_reconnect_attempt_time = now
        candidates = self.device_candidates()
        if not candidates:
            self.log_error_throttled('Waiting for controller device')
            return

        last_error = None
        for device_path, device_type in candidates:
            try:
                self.device_fd = os.open(device_path, os.O_RDONLY | os.O_NONBLOCK)
            except OSError as exc:
                last_error = exc
                continue

            self.active_device_path = device_path
            self.active_device_type = device_type
            self.axes = []
            self.buttons = []
            self.event_button_map = {}
            self.get_logger().info(
                f'Controller connected: {device_path} ({device_type})'
            )
            return

        self.log_error_throttled(f'Waiting for controller device: {last_error}')

    def device_candidates(self):
        if self.device_path != 'auto':
            return [(self.device_path, self.resolve_device_type(self.device_path))]

        candidates = []
        if self.device_type in ('auto', 'js'):
            candidates.extend((path, 'js') for path in sorted(glob('/dev/input/js*')))
        if self.device_type in ('auto', 'event'):
            by_id = sorted(glob('/dev/input/by-id/*event-joystick'))
            by_path = sorted(glob('/dev/input/by-path/*event-joystick'))
            events = sorted(glob('/dev/input/event*'))
            candidates.extend((path, 'event') for path in by_id + by_path + events)
        return candidates

    def resolve_device_type(self, device_path):
        if self.device_type != 'auto':
            return self.device_type
        if '/js' in device_path or os.path.basename(device_path).startswith('js'):
            return 'js'
        return 'event'

    def close_device(self):
        if self.device_fd is not None:
            try:
                os.close(self.device_fd)
            except OSError:
                pass
            self.device_fd = None

    def read_pending_js_events(self):
        while True:
            try:
                data = os.read(self.device_fd, JS_EVENT_SIZE)
            except BlockingIOError:
                break

            if not data:
                raise OSError('device returned no data')
            if len(data) != JS_EVENT_SIZE:
                continue

            _, value, event_type, number = struct.unpack(JS_EVENT_FORMAT, data)
            event_type = event_type & ~JS_EVENT_INIT

            if event_type == JS_EVENT_AXIS:
                self.ensure_axis(number)
                self.axes[number] = self.apply_deadzone(value / JS_AXIS_MAX)
            elif event_type == JS_EVENT_BUTTON:
                self.ensure_button(number)
                self.buttons[number] = 1 if value else 0

    def read_pending_event_events(self):
        while True:
            try:
                data = os.read(self.device_fd, INPUT_EVENT_SIZE)
            except BlockingIOError:
                break

            if not data:
                raise OSError('device returned no data')
            if len(data) != INPUT_EVENT_SIZE:
                continue

            _, _, event_type, code, value = struct.unpack(INPUT_EVENT_FORMAT, data)
            if event_type == EV_SYN:
                continue
            if event_type == EV_ABS:
                index = self.event_axis_map.setdefault(code, len(self.event_axis_map))
                self.ensure_axis(index)
                self.axes[index] = self.apply_deadzone(value / self.event_axis_max)
            elif event_type == EV_KEY:
                index = self.event_button_map.setdefault(code, len(self.event_button_map))
                if code >= DEFAULT_BUTTON_BASE:
                    index = self.event_button_map[code]
                self.ensure_button(index)
                self.buttons[index] = 1 if value else 0

    def apply_deadzone(self, value):
        value = max(-1.0, min(1.0, float(value)))
        return 0.0 if abs(value) < self.deadzone else value

    def publish_joy(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'controller'
        msg.axes = list(self.axes)
        msg.buttons = list(self.buttons)
        self.publisher.publish(msg)

    def ensure_axis(self, index):
        while len(self.axes) <= index:
            self.axes.append(0.0)

    def ensure_button(self, index):
        while len(self.buttons) <= index:
            self.buttons.append(0)

    def log_error_throttled(self, message):
        now = time.monotonic()
        if now - self.last_error_log_time >= 5.0:
            self.get_logger().error(message)
            self.last_error_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
