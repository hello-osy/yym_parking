"""Exclusive bidirectional Arduino serial owner inside sensor_topic."""

import math
import time
from glob import glob

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16, Int16MultiArray

from .serial_port import open_serial


MAX_RX_BUFFER_BYTES = 4096


def parse_ultrasonic_line(line, sensor_count=6):
    """Return a complete U frame in metres, or None for a damaged frame."""
    parts = [part.strip() for part in line.split(',')]
    if len(parts) != sensor_count + 1 or parts[0] != 'U':
        return None

    values = []
    try:
        for raw_value in parts[1:]:
            value = float(raw_value)
            if not math.isfinite(value) or value < 0.0:
                value = math.inf
            values.append(value)
    except ValueError:
        return None
    return values


def parse_rotation_line(line):
    """Return the A1 raw value from a complete R frame, else None."""
    parts = [part.strip() for part in line.split(',')]
    if len(parts) != 4 or parts[0] != 'R':
        return None
    try:
        raw_value = int(parts[1])
        voltage = float(parts[2])
        percent = int(parts[3])
    except ValueError:
        return None
    if (
        not 0 <= raw_value <= 1023
        or not math.isfinite(voltage)
        or not 0 <= percent <= 100
    ):
        return None
    return raw_value


class ArduinoCommunicationNode(Node):
    """Own the Arduino port, prioritize motor TX, then process sensor RX."""

    def __init__(self):
        super().__init__('arduino_communication_node')
        self.declare_parameter('port', 'auto')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('arduino_boot_delay', 2.0)
        self.declare_parameter('command_topic', '/arduino/motor_command')
        self.declare_parameter('ultrasonic_raw_topic', '/arduino/ultrasonic_raw')
        self.declare_parameter(
            'steering_raw_topic', '/arduino/steering_raw'
        )
        self.declare_parameter('sensor_count', 6)
        self.declare_parameter('command_rate_hz', 20.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('max_steer_pwm', 150)
        self.declare_parameter('max_drive_pwm', 230)
        self.declare_parameter('debug_serial_lines', False)

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.arduino_boot_delay = max(
            0.0, float(self.get_parameter('arduino_boot_delay').value)
        )
        self.sensor_count = max(
            1, int(self.get_parameter('sensor_count').value)
        )
        self.command_rate_hz = max(
            1.0, float(self.get_parameter('command_rate_hz').value)
        )
        self.command_timeout_sec = max(
            0.05, float(self.get_parameter('command_timeout_sec').value)
        )
        self.max_steer_pwm = max(
            0, min(255, int(self.get_parameter('max_steer_pwm').value))
        )
        self.max_drive_pwm = max(
            0, min(255, int(self.get_parameter('max_drive_pwm').value))
        )
        self.debug_serial_lines = bool(
            self.get_parameter('debug_serial_lines').value
        )

        self.serial = None
        self.active_port = ''
        self.last_open_attempt = 0.0
        self.last_error_log = 0.0
        self.rx_buffer = bytearray()
        self.command = (0, 0)
        self.last_command_received = None

        self.ultrasonic_publisher = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter('ultrasonic_raw_topic').value),
            10,
        )
        self.steering_raw_publisher = self.create_publisher(
            Int16,
            str(self.get_parameter('steering_raw_topic').value),
            10,
        )
        self.create_subscription(
            Int16MultiArray,
            str(self.get_parameter('command_topic').value),
            self.command_callback,
            10,
        )
        self.create_timer(1.0 / self.command_rate_hz, self.poll)
        self.get_logger().info(
            'Arduino communication node owns ttyACM serial: motor TX first, '
            'ultrasonic RX second, bridge watchdog enabled'
        )

    def command_callback(self, msg):
        steer = int(msg.data[0]) if len(msg.data) > 0 else 0
        drive = int(msg.data[1]) if len(msg.data) > 1 else 0
        self.command = (
            max(-self.max_steer_pwm, min(self.max_steer_pwm, steer)),
            max(-self.max_drive_pwm, min(self.max_drive_pwm, drive)),
        )
        self.last_command_received = time.monotonic()

    def poll(self):
        if self.serial is None and not self.open():
            return

        command = self.command
        if (
            self.last_command_received is None
            or time.monotonic() - self.last_command_received
            > self.command_timeout_sec
        ):
            command = (0, 0)

        # Safety-critical command transmission always precedes sensor reads.
        if not self.write_command(*command):
            return
        self.read_available()

    def open(self):
        now = time.monotonic()
        if now - self.last_open_attempt < 1.0:
            return False
        self.last_open_attempt = now

        candidates = (
            [self.port]
            if self.port != 'auto'
            else sorted(glob('/dev/ttyACM*'))
        )
        if not candidates:
            self.log_error_throttled('Waiting for Arduino /dev/ttyACM*')
            return False

        last_error = None
        for port in candidates:
            try:
                self.serial = open_serial(
                    port,
                    self.baudrate,
                    timeout=0.0,
                    write_timeout=0.05,
                )
                self.active_port = port
                self.rx_buffer.clear()
                self.get_logger().info(
                    f'Arduino communication connected on {port}'
                )
                if self.arduino_boot_delay > 0.0:
                    time.sleep(self.arduino_boot_delay)
                if hasattr(self.serial, 'reset_input_buffer'):
                    self.serial.reset_input_buffer()
                self.write_command(0, 0)
                return True
            except Exception as exc:
                last_error = exc
                self.close()

        self.log_error_throttled(f'Waiting for Arduino serial: {last_error}')
        return False

    def write_command(self, steer, drive):
        try:
            self.serial.write(f'{steer} {drive}\n'.encode())
            self.serial.flush()
            return True
        except Exception as exc:
            self.log_error_throttled(f'Arduino serial write failed: {exc}')
            self.close()
            return False

    def read_available(self):
        try:
            waiting = int(getattr(self.serial, 'in_waiting', 0))
            if waiting <= 0:
                return
            self.rx_buffer.extend(self.serial.read(min(waiting, 1024)))
        except Exception as exc:
            self.log_error_throttled(f'Arduino serial read failed: {exc}')
            self.close()
            return

        if len(self.rx_buffer) > MAX_RX_BUFFER_BYTES:
            self.rx_buffer.clear()
            return

        while b'\n' in self.rx_buffer:
            raw_line, _, remainder = self.rx_buffer.partition(b'\n')
            self.rx_buffer = bytearray(remainder)
            line = raw_line.decode(errors='ignore').strip()
            if not line:
                continue
            values = parse_ultrasonic_line(line, self.sensor_count)
            if values is not None:
                message = Float32MultiArray()
                message.data = values
                self.ultrasonic_publisher.publish(message)
                continue

            steering_raw = parse_rotation_line(line)
            if steering_raw is not None:
                message = Int16()
                message.data = steering_raw
                self.steering_raw_publisher.publish(message)
            elif self.debug_serial_lines:
                self.get_logger().info(f'Ignored Arduino line: {line}')

    def close(self):
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None
        self.active_port = ''
        self.rx_buffer.clear()

    def destroy_node(self):
        if self.serial is not None:
            self.write_command(0, 0)
        self.close()
        return super().destroy_node()

    def log_error_throttled(self, message):
        now = time.monotonic()
        if now - self.last_error_log >= 5.0:
            self.get_logger().error(message)
            self.last_error_log = now


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoCommunicationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
