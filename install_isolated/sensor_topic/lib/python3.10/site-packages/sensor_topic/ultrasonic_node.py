"""Convert Arduino raw ultrasonic arrays into the existing Range topics."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray


class UltrasonicNode(Node):
    """Publish Range topics without directly accessing the Arduino serial port."""

    def __init__(self):
        super().__init__('ultrasonic_node')
        self.declare_parameter(
            'raw_topic',
            '/arduino/ultrasonic_raw',
        )
        self.declare_parameter(
            'sensor_names',
            ['1', '2', '3', '4', '5', '6'],
        )
        self.declare_parameter('frame_prefix', 'ultrasonic_')
        self.declare_parameter('field_of_view', 0.26)
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('max_range', 4.0)

        self.raw_topic = str(self.get_parameter('raw_topic').value)
        self.sensor_names = list(self.get_parameter('sensor_names').value)
        self.frame_prefix = str(self.get_parameter('frame_prefix').value)
        self.field_of_view = float(self.get_parameter('field_of_view').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)

        self.range_publishers = {
            name: self.create_publisher(
                Range,
                f'/ultrasonic/range_{name}',
                10,
            )
            for name in self.sensor_names
        }
        self.array_publisher = self.create_publisher(
            Float32MultiArray,
            '/ultrasonic/ranges',
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            self.raw_topic,
            self.raw_callback,
            10,
        )
        self.get_logger().info(
            f'Subscribing {self.raw_topic}; no direct serial access'
        )

    def raw_callback(self, msg):
        # The bridge only emits complete frames, but preserve the boundary
        # check so malformed external publishers cannot create partial topics.
        if len(msg.data) != len(self.sensor_names):
            return
        values = [self.normalize_distance(value) for value in msg.data]
        self.publish(values)

    @staticmethod
    def normalize_distance(value):
        value = float(value)
        if not math.isfinite(value) or value < 0.0:
            return math.inf
        return value

    def publish(self, values):
        stamp = self.get_clock().now().to_msg()
        for name, distance in zip(self.sensor_names, values):
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_prefix + name
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = self.field_of_view
            msg.min_range = self.min_range
            msg.max_range = self.max_range
            msg.range = float(distance)
            self.range_publishers[name].publish(msg)

        array_msg = Float32MultiArray()
        array_msg.data = [float(value) for value in values]
        self.array_publisher.publish(array_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
