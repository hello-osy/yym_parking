import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class UltrasonicViewerNode(Node):
    """Render ultrasonic range readings as simple direction bars."""

    def __init__(self):
        super().__init__('ultrasonic_viewer_node')
        self.declare_parameter('sensor_names', ['1', '2', '3', '4', '5', '6'])
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('window_name', 'ultrasonic_ranges')
        self.declare_parameter('stale_timeout', 1.0)

        self.sensor_names = list(self.get_parameter('sensor_names').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.window_name = self.get_parameter('window_name').value
        self.stale_timeout = float(self.get_parameter('stale_timeout').value)
        self.ranges = {name: math.inf for name in self.sensor_names}
        self.last_update = {name: None for name in self.sensor_names}

        for name in self.sensor_names:
            self.create_subscription(
                Range,
                f'/ultrasonic/range_{name}',
                lambda msg, sensor=name: self.range_callback(sensor, msg),
                10,
            )
        self.timer = self.create_timer(0.05, self.draw)

    def range_callback(self, name, msg):
        self.ranges[name] = msg.range
        self.last_update[name] = time.monotonic()

    def draw(self):
        width = 700
        height = 430
        image = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(
            image,
            'Ultrasonic ranges',
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        bar_x = 160
        bar_w = 360
        for index, name in enumerate(self.sensor_names):
            y = 80 + index * 55
            distance = self.ranges.get(name, math.inf)
            updated_at = self.last_update.get(name)
            is_stale = updated_at is not None and time.monotonic() - updated_at > self.stale_timeout
            ratio = 0.0 if not math.isfinite(distance) else min(distance / self.max_range, 1.0)
            color = (0, 80, 255) if ratio < 0.25 else (0, 220, 0)
            if updated_at is None or is_stale:
                color = (80, 80, 80)

            cv2.putText(
                image,
                name,
                (25, y + 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 220, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.rectangle(image, (bar_x, y - 18), (bar_x + bar_w, y + 18), (0, 80, 0), 1)
            cv2.rectangle(
                image,
                (bar_x, y - 18),
                (bar_x + int(bar_w * ratio), y + 18),
                color,
                -1,
            )
            if updated_at is None:
                label = 'no topic'
            elif is_stale:
                label = 'stale'
            elif not math.isfinite(distance):
                label = 'no echo'
            else:
                label = f'{distance:.2f} m'
            cv2.putText(
                image,
                label,
                (bar_x + bar_w + 20, y + 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 220, 0),
                1,
                cv2.LINE_AA,
            )

        cv2.imshow(self.window_name, image)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
