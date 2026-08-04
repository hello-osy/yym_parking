import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarViewerNode(Node):
    """Render LaserScan as a radar-style top-down display."""

    def __init__(self):
        super().__init__('lidar_viewer_node')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('window_name', 'lidar_radar')
        # 가까운 주차 장애물을 크게 보기 위한 시각화 최대 거리.
        self.declare_parameter('max_range_m', 2.0)
        self.declare_parameter('image_size', 800)
        self.declare_parameter('range_ring_step_m', 1.0)
        # Visualisation shows every valid return.  Parking control applies
        # its own rear/side filtering independently.
        self.declare_parameter('rear_quadrants_only', False)

        self.window_name = self.get_parameter('window_name').value
        self.max_range = float(self.get_parameter('max_range_m').value)
        self.image_size = int(self.get_parameter('image_size').value)
        self.ring_step = float(self.get_parameter('range_ring_step_m').value)
        self.rear_quadrants_only = bool(
            self.get_parameter('rear_quadrants_only').value
        )
        self.latest_scan = None

        self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            10,
        )
        self.timer = self.create_timer(0.033, self.draw)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def draw(self):
        size = self.image_size
        center = size // 2
        scale = (size * 0.45) / self.max_range
        image = np.zeros((size, size, 3), dtype=np.uint8)

        for radius_m in np.arange(self.ring_step, self.max_range + 0.01, self.ring_step):
            radius_px = int(radius_m * scale)
            cv2.circle(image, (center, center), radius_px, (0, 90, 0), 1)
            cv2.putText(
                image,
                f'{radius_m:.0f}m',
                (center + 5, center - radius_px - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 180, 0),
                1,
                cv2.LINE_AA,
            )

        self.draw_bearing_guides(image, center, size)
        cv2.circle(image, (center, center), 7, (0, 255, 0), -1)
        cv2.putText(
            image,
            'CAR',
            (center + 12, center + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

        if self.latest_scan is not None:
            self.draw_scan(image, center, scale)
        else:
            cv2.putText(
                image,
                'Waiting for /scan',
                (25, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 180, 0),
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            image,
            'REAR 0 | RIGHT +90 | FRONT +/-180 | LEFT -90',
            (25, size - 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 220, 220),
            1,
            cv2.LINE_AA,
        )

        cv2.imshow(self.window_name, image)
        cv2.waitKey(1)

    def draw_bearing_guides(self, image, center, size):
        guide_color = (0, 90, 0)
        label_color = (0, 220, 220)
        radius = int(size * 0.45)

        # Bearings increase counter-clockwise from the vehicle rear.
        bearings = [
            (0, '0 deg REAR', (center + 12, center + radius - 12)),
            (90, '+90 deg RIGHT', (center + radius - 165, center - 14)),
            (-90, '-90 deg LEFT', (center - radius + 18, center - 14)),
            (180, '+/-180 deg FRONT', (center - 100, center - radius + 24)),
        ]

        cv2.line(image, (center, 20), (center, size - 20), guide_color, 1)
        cv2.line(image, (20, center), (size - 20, center), guide_color, 1)
        cv2.arrowedLine(
            image,
            (center, center),
            (center, 35),
            label_color,
            2,
            tipLength=0.08,
        )

        for angle_deg, label, label_pos in bearings:
            angle = math.radians(angle_deg)
            x = int(center + math.sin(angle) * radius)
            y = int(center + math.cos(angle) * radius)
            cv2.circle(image, (x, y), 5, label_color, -1)
            cv2.putText(
                image,
                label,
                label_pos,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                label_color,
                1,
                cv2.LINE_AA,
            )

        cv2.ellipse(
            image,
            (center, center),
            (70, 70),
            0,
            250,
            110,
            label_color,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            '+ CCW / RIGHT',
            (center + 78, center + 92),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            label_color,
            1,
            cv2.LINE_AA,
        )

    def draw_scan(self, image, center, scale):
        msg = self.latest_scan
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > min(msg.range_max, self.max_range):
                continue

            angle = msg.angle_min + index * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            # Keep this optional filter only for focused debugging.  Its
            # default is off so the radar always exposes the complete scan.
            if self.rear_quadrants_only and abs(angle) > math.pi / 2:
                continue
            x = int(center + math.sin(angle) * distance * scale)
            y = int(center + math.cos(angle) * distance * scale)
            intensity = 120
            if index < len(msg.intensities):
                intensity = min(255, 80 + int(msg.intensities[index] * 4))
            cv2.circle(image, (x, y), 2, (0, intensity, 30), -1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
