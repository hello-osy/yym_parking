import math
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from sensor_topic.serial_port import open_serial


SYNC_BYTE = 0xA5
CMD_SCAN = 0x20
CMD_STOP = 0x25
SCAN_DESCRIPTOR = bytes([0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81])


class LidarNode(Node):
    """Publish LaserScan using the vehicle's rear-zero bearing convention.

    Bearings increase counter-clockwise from the vehicle rear:
    rear=0, right=+pi/2, front=+/-pi, and left=-pi/2.
    """

    def __init__(self):
        super().__init__('lidar_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('topic_name', '/scan')
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('angle_increment_deg', 1.0)
        self.declare_parameter('publish_rate', 10.0)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.angle_increment = math.radians(
            float(self.get_parameter('angle_increment_deg').value)
        )
        self.bin_count = int(round((2.0 * math.pi) / self.angle_increment))
        self.publish_period = 1.0 / float(self.get_parameter('publish_rate').value)
        self.publisher = self.create_publisher(
            LaserScan,
            self.get_parameter('topic_name').value,
            10,
        )

        self.serial = None
        self.current_ranges = [math.inf] * self.bin_count
        self.current_intensities = [0.0] * self.bin_count
        self.last_publish = time.monotonic()
        self.last_open_attempt = 0.0
        self.timer = self.create_timer(0.002, self.poll)

    def destroy_node(self):
        self.stop_scan()
        self.close()
        super().destroy_node()

    def poll(self):
        if self.serial is None and not self.open_lidar():
            return

        try:
            self.read_measurements(max_packets=80)
        except Exception as exc:
            self.get_logger().warn(
                f'RPLidar read failed, reopening {self.port}: {exc}',
                throttle_duration_sec=2.0,
            )
            self.close()

    def open_lidar(self):
        now = time.monotonic()
        if now - self.last_open_attempt < 1.0:
            return False
        self.last_open_attempt = now

        try:
            self.serial = open_serial(
                self.port,
                self.baudrate,
                timeout=0.02,
                write_timeout=0.2,
                dtr=False,
            )
            self.start_scan()
            self.get_logger().info(f'RPLidar connected on {self.port}')
            return True
        except Exception as exc:
            self.get_logger().warn(
                f'Waiting for RPLidar A1 on {self.port}: {exc}',
                throttle_duration_sec=5.0,
            )
            self.close()
            return False

    def start_scan(self):
        self.send_command(CMD_STOP)
        time.sleep(0.1)
        self.serial.reset_input_buffer()
        self.send_command(CMD_SCAN)
        time.sleep(0.2)
        descriptor = self.serial.read(len(SCAN_DESCRIPTOR))
        if descriptor != SCAN_DESCRIPTOR:
            self.get_logger().warn(
                f'Unexpected RPLidar descriptor: {descriptor!r}',
                throttle_duration_sec=5.0,
            )

    def stop_scan(self):
        if self.serial is not None:
            try:
                self.send_command(CMD_STOP)
            except Exception:
                pass

    def send_command(self, command):
        self.serial.write(bytes([SYNC_BYTE, command]))
        self.serial.flush()

    def close(self):
        if self.serial is not None:
            try:
                self.serial.close()
            finally:
                self.serial = None

    def read_measurements(self, max_packets):
        for _ in range(max_packets):
            data = self.serial.read(5)
            if len(data) < 5:
                return
            parsed = self.parse_measurement(data)
            if parsed is None:
                continue
            angle_rad, distance_m, quality = parsed
            signed_angle = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
            index = int(
                round((signed_angle + math.pi) / self.angle_increment)
            ) % self.bin_count
            if self.range_min <= distance_m <= self.range_max:
                self.current_ranges[index] = distance_m
                self.current_intensities[index] = float(quality)

        now = time.monotonic()
        if now - self.last_publish >= self.publish_period:
            self.publish_scan()
            self.current_ranges = [math.inf] * self.bin_count
            self.current_intensities = [0.0] * self.bin_count
            self.last_publish = now

    def parse_measurement(self, data):
        b0, b1, b2, b3, b4 = struct.unpack('<BBBBB', data)
        start_flag = b0 & 0x01
        inverse_start_flag = (b0 >> 1) & 0x01
        check_bit = b1 & 0x01
        if start_flag == inverse_start_flag or check_bit != 1:
            return None

        quality = b0 >> 2
        angle_q6 = ((b2 << 7) | (b1 >> 1))
        distance_q2 = (b4 << 8) | b3
        angle_deg = (angle_q6 / 64.0) % 360.0
        distance_m = (distance_q2 / 4.0) / 1000.0
        return math.radians(angle_deg), distance_m, quality

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = -math.pi
        msg.angle_max = math.pi - self.angle_increment
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = self.publish_period
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = list(self.current_ranges)
        msg.intensities = list(self.current_intensities)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
