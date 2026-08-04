import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class ControllerViewerNode(Node):
    """Render sensor_msgs/Joy axes and buttons in an OpenCV window."""

    def __init__(self):
        super().__init__('controller_viewer_node')
        self.declare_parameter('joy_topic', '/manual_controller/joy')
        self.declare_parameter('window_name', 'controller')
        self.declare_parameter('stale_timeout', 1.0)

        self.window_name = self.get_parameter('window_name').value
        self.stale_timeout = float(self.get_parameter('stale_timeout').value)
        self.latest_msg = None
        self.latest_time = 0.0

        self.create_subscription(
            Joy,
            self.get_parameter('joy_topic').value,
            self.joy_callback,
            10,
        )
        self.timer = self.create_timer(0.033, self.draw)

    def joy_callback(self, msg):
        self.latest_msg = msg
        self.latest_time = time.monotonic()

    def draw(self):
        image = np.zeros((520, 760, 3), dtype=np.uint8)
        self.put_text(image, 'Controller /manual_controller/joy', 24, 34, 0.75)

        if self.latest_msg is None:
            self.put_text(image, 'Waiting for controller topic...', 24, 82, 0.7)
            cv2.imshow(self.window_name, image)
            cv2.waitKey(1)
            return

        age = time.monotonic() - self.latest_time
        status = 'STALE' if age > self.stale_timeout else 'LIVE'
        color = (0, 120, 255) if status == 'STALE' else (0, 220, 80)
        self.put_text(image, f'{status}  age={age:.2f}s', 24, 72, 0.65, color)

        self.draw_axes(image, self.latest_msg.axes)
        self.draw_buttons(image, self.latest_msg.buttons)

        cv2.imshow(self.window_name, image)
        cv2.waitKey(1)

    def draw_axes(self, image, axes):
        self.put_text(image, 'Axes', 24, 120, 0.7)
        x0 = 110
        y0 = 145
        width = 260
        center = x0 + width // 2

        for index, value in enumerate(axes[:8]):
            y = y0 + index * 36
            value = max(-1.0, min(1.0, float(value)))
            end = int(center + value * (width // 2))

            self.put_text(image, f'{index}: {value:+.2f}', 24, y + 7, 0.52)
            cv2.line(image, (x0, y), (x0 + width, y), (70, 70, 70), 2)
            cv2.line(image, (center, y - 8), (center, y + 8), (150, 150, 150), 1)
            cv2.line(image, (center, y), (end, y), (0, 220, 255), 8)

    def draw_buttons(self, image, buttons):
        self.put_text(image, 'Buttons', 420, 120, 0.7)
        x0 = 420
        y0 = 150
        gap = 42

        for index, pressed in enumerate(buttons[:16]):
            col = index % 4
            row = index // 4
            x = x0 + col * 78
            y = y0 + row * gap
            color = (0, 220, 80) if pressed else (70, 70, 70)
            cv2.circle(image, (x, y), 13, color, -1)
            self.put_text(image, str(index), x + 20, y + 5, 0.5)

    def put_text(self, image, text, x, y, scale, color=(230, 230, 230)):
        cv2.putText(
            image,
            text,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            1,
            cv2.LINE_AA,
        )

    def destroy_node(self):
        cv2.destroyWindow(self.window_name)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerViewerNode()
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
