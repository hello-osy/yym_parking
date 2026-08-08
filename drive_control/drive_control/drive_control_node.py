"""Convert target steering/speed into Arduino-ready motor PWM commands.

This node never opens a serial port. sensor_topic's Arduino communication node
exclusively owns the device and consumes /arduino/motor_command.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Int16MultiArray


MOTOR_CONTROL_TOPIC = '/motor_control'
ARDUINO_COMMAND_TOPIC = '/arduino/motor_command'
STEERING_RAW_TOPIC = '/arduino/steering_raw'
STEERING_CALIBRATION_REQUEST_TOPIC = '/steering/calibrate_center'
STEERING_CALIBRATION_STATUS_TOPIC = '/steering/calibration'
STEERING_CALIBRATION_HALF_SPAN = 100
COMMAND_RATE = 20.0
INPUT_TIMEOUT = 0.5
STEERING_FEEDBACK_TIMEOUT = 0.5

MAX_DRIVE_PWM = 230
STEER_PWM = 150
STEER_MAX_ANGLE_DEG = 45.0
STEER_ANGLE_TOLERANCE_DEG = 1.0
STEER_RAW_LEFT = 550
STEER_RAW_CENTER = 480
STEER_RAW_RIGHT = 410
STEER_PID_KP = 10.0
STEER_PID_KI = 0.0
STEER_PID_KD = 0.5
STEER_PID_INTEGRAL_LIMIT_PWM = 30.0
STEER_PID_DERIVATIVE_FILTER_ALPHA = 0.25
STEER_MIN_PWM = 40

# The parking controller publishes an already-calculated steering target.  Its
# low-gain proportional loop avoids amplifying a small target update into a
# large steering-motor correction while retaining raw-angle feedback.
PARKING_STEER_PID_KP = 4.0
PARKING_STEER_PID_KI = 0.0
PARKING_STEER_PID_KD = 0.0


class DriveControlNode(Node):
    """Translate /motor_control targets into steering and drive PWM."""

    def __init__(self):
        super().__init__('drive_control_node')
        # parking.launch.py remaps this node to ``parking_drive_control``.
        self.parking_mode = self.get_name() == 'parking_drive_control'
        self.declare_parameter('motor_control_topic', MOTOR_CONTROL_TOPIC)
        self.declare_parameter('arduino_command_topic', ARDUINO_COMMAND_TOPIC)
        self.declare_parameter('steering_raw_topic', STEERING_RAW_TOPIC)
        self.declare_parameter('command_rate_hz', COMMAND_RATE)
        self.declare_parameter('input_timeout_sec', INPUT_TIMEOUT)
        self.declare_parameter(
            'steering_feedback_timeout_sec',
            STEERING_FEEDBACK_TIMEOUT,
        )
        self.declare_parameter('max_drive_pwm', MAX_DRIVE_PWM)
        self.declare_parameter('steer_pwm', STEER_PWM)
        self.declare_parameter('steer_max_angle_deg', STEER_MAX_ANGLE_DEG)
        self.declare_parameter(
            'steer_angle_tolerance_deg',
            STEER_ANGLE_TOLERANCE_DEG,
        )
        self.declare_parameter('steer_pid_kp', STEER_PID_KP)
        self.declare_parameter('steer_pid_ki', STEER_PID_KI)
        self.declare_parameter('steer_pid_kd', STEER_PID_KD)
        self.declare_parameter(
            'steer_pid_integral_limit_pwm',
            STEER_PID_INTEGRAL_LIMIT_PWM,
        )
        self.declare_parameter(
            'steer_pid_derivative_filter_alpha',
            STEER_PID_DERIVATIVE_FILTER_ALPHA,
        )
        self.declare_parameter('steer_min_pwm', STEER_MIN_PWM)

        self.motor_control_topic = str(
            self.get_parameter('motor_control_topic').value
        )
        self.arduino_command_topic = str(
            self.get_parameter('arduino_command_topic').value
        )
        self.steering_raw_topic = str(
            self.get_parameter('steering_raw_topic').value
        )
        self.command_rate_hz = max(
            1.0, float(self.get_parameter('command_rate_hz').value)
        )
        self.input_timeout = max(
            0.05, float(self.get_parameter('input_timeout_sec').value)
        )
        self.steering_feedback_timeout = max(
            0.05,
            float(
                self.get_parameter(
                    'steering_feedback_timeout_sec'
                ).value
            ),
        )
        self.max_drive_pwm = max(
            0, min(255, int(self.get_parameter('max_drive_pwm').value))
        )
        self.steer_pwm = max(
            0, min(255, abs(int(self.get_parameter('steer_pwm').value)))
        )
        self.steer_max_angle_deg = max(
            1.0, abs(float(self.get_parameter('steer_max_angle_deg').value))
        )
        self.steer_angle_tolerance_deg = max(
            0.0,
            float(self.get_parameter('steer_angle_tolerance_deg').value),
        )
        # Single source of truth for steering calibration. Change only the
        # STEER_RAW_* constants at the top of this file.
        self.steer_raw_left = STEER_RAW_LEFT
        self.steer_raw_center = STEER_RAW_CENTER
        self.steer_raw_right = STEER_RAW_RIGHT
        self.steer_pid_kp = max(
            0.0, float(self.get_parameter('steer_pid_kp').value)
        )
        self.steer_pid_ki = max(
            0.0, float(self.get_parameter('steer_pid_ki').value)
        )
        self.steer_pid_kd = max(
            0.0, float(self.get_parameter('steer_pid_kd').value)
        )
        self.steer_pid_integral_limit_pwm = max(
            0.0,
            float(
                self.get_parameter(
                    'steer_pid_integral_limit_pwm'
                ).value
            ),
        )
        self.steer_pid_derivative_filter_alpha = max(
            0.0,
            min(
                1.0,
                float(
                    self.get_parameter(
                        'steer_pid_derivative_filter_alpha'
                    ).value
                ),
            ),
        )
        self.steer_min_pwm = max(
            0,
            min(
                self.steer_pwm,
                abs(int(self.get_parameter('steer_min_pwm').value)),
            ),
        )
        if self.parking_mode:
            # parking.launch.py gives this node the parking_drive_control
            # name. Keep parking PID fixed here even if generic controller
            # parameters are supplied by another launch/config file.
            self.steer_pid_kp = PARKING_STEER_PID_KP
            self.steer_pid_ki = PARKING_STEER_PID_KI
            self.steer_pid_kd = PARKING_STEER_PID_KD
        if not (
            self.steer_raw_left
            > self.steer_raw_center
            > self.steer_raw_right
        ):
            self.get_logger().warn(
                'Invalid steering raw calibration; using 550/480/40'
            )
            self.steer_raw_left = STEER_RAW_LEFT
            self.steer_raw_center = STEER_RAW_CENTER
            self.steer_raw_right = STEER_RAW_RIGHT

        self.last_input_time = None
        self.last_steering_feedback_time = None
        self.drive_pwm = 0
        self.steering_enabled = True
        self.target_steer_angle_deg = 0.0
        self.steer_angle_deg = 0.0
        self.steer_raw_value = None
        self.steer_angle_velocity_deg_per_sec = 0.0
        self.pid_integral_error = 0.0
        self.last_pid_update_time = None

        self.command_publisher = self.create_publisher(
            Int16MultiArray,
            self.arduino_command_topic,
            10,
        )
        self.calibration_status_publisher = self.create_publisher(
            Int16MultiArray,
            STEERING_CALIBRATION_STATUS_TOPIC,
            10,
        )
        self.create_subscription(
            Int16MultiArray,
            self.motor_control_topic,
            self.motor_control_callback,
            10,
        )
        self.create_subscription(
            Int16,
            self.steering_raw_topic,
            self.steering_feedback_callback,
            10,
        )
        self.create_subscription(
            Int16,
            STEERING_CALIBRATION_REQUEST_TOPIC,
            self.center_calibration_callback,
            10,
        )
        self.create_timer(1.0 / self.command_rate_hz, self.timer_callback)
        self.get_logger().info(
            f'{self.motor_control_topic} target -> '
            f'{self.arduino_command_topic} PWM using '
            f'{self.steering_raw_topic} closed-loop feedback; '
            f'raw left/center/right='
            f'{self.steer_raw_left}/{self.steer_raw_center}/'
            f'{self.steer_raw_right}; PID='
            f'{self.steer_pid_kp:.2f}/{self.steer_pid_ki:.2f}/'
            f'{self.steer_pid_kd:.2f}'
        )

    def motor_control_callback(self, msg):
        self.last_input_time = self.get_clock().now()
        steer = int(msg.data[0]) if len(msg.data) > 0 else 0
        speed = int(msg.data[1]) if len(msg.data) > 1 else 0
        # Optional third field: 0 disables steering-motor output while still
        # allowing drive PWM. Existing two-field publishers remain unchanged.
        steering_enabled = (
            bool(msg.data[2]) if len(msg.data) > 2 else True
        )
        self.drive_pwm = self.limit_drive_pwm(speed)

        if not steering_enabled:
            if self.steering_enabled:
                self.reset_pid_controller()
            self.steering_enabled = False
            return
        self.steering_enabled = True

        previous_error = (
            self.target_steer_angle_deg - self.steer_angle_deg
        )
        self.target_steer_angle_deg = self.clamp_steer_angle(steer)
        next_error = self.target_steer_angle_deg - self.steer_angle_deg
        if previous_error * next_error < 0.0:
            self.pid_integral_error = 0.0

    def steering_feedback_callback(self, msg):
        raw_value = int(msg.data)
        if not 0 <= raw_value <= 1023:
            return
        now = self.get_clock().now()
        next_angle = self.raw_to_steer_angle(raw_value)
        if self.last_steering_feedback_time is not None:
            feedback_dt = (
                now - self.last_steering_feedback_time
            ).nanoseconds / 1e9
            if (
                feedback_dt > 0.0
                and feedback_dt
                <= self.steering_feedback_timeout * 2.0
            ):
                raw_velocity = (
                    next_angle - self.steer_angle_deg
                ) / feedback_dt
                alpha = self.steer_pid_derivative_filter_alpha
                self.steer_angle_velocity_deg_per_sec = (
                    alpha * raw_velocity
                    + (1.0 - alpha)
                    * self.steer_angle_velocity_deg_per_sec
                )
            else:
                self.steer_angle_velocity_deg_per_sec = 0.0
        else:
            self.steer_angle_velocity_deg_per_sec = 0.0
        self.steer_raw_value = raw_value
        self.steer_angle_deg = next_angle
        self.last_steering_feedback_time = now

    def center_calibration_callback(self, msg):
        """Use the current physical wheel position as zero steering."""
        if int(msg.data) == 0:
            return
        now = self.get_clock().now()
        if (
            self.steer_raw_value is None
            or self.is_steering_feedback_stale(now)
        ):
            self.get_logger().warning(
                'Center calibration rejected: steering feedback unavailable'
            )
            return

        center = int(self.steer_raw_value)
        right = center - STEERING_CALIBRATION_HALF_SPAN
        left = center + STEERING_CALIBRATION_HALF_SPAN
        if right < 0 or left > 1023:
            self.get_logger().error(
                f'Center calibration rejected: raw center {center} cannot '
                f'use +/-{STEERING_CALIBRATION_HALF_SPAN}'
            )
            return

        self.steer_raw_left = left
        self.steer_raw_center = center
        self.steer_raw_right = right
        self.target_steer_angle_deg = 0.0
        self.steer_angle_deg = 0.0
        self.steer_angle_velocity_deg_per_sec = 0.0
        self.reset_pid_controller()

        status = Int16MultiArray()
        status.data = [left, center, right]
        self.calibration_status_publisher.publish(status)
        self.get_logger().info(
            'Steering center calibrated from current wheel position: '
            f'raw left/center/right={left}/{center}/{right}'
        )

    def timer_callback(self):
        now = self.get_clock().now()

        if self.is_input_stale(now):
            self.reset_pid_controller()
            steer_pwm, drive_pwm = 0, 0
        elif not self.steering_enabled:
            # The caller explicitly requested free steering: never energize
            # the steering motor, and pass only the requested drive PWM.
            self.reset_pid_controller()
            steer_pwm, drive_pwm = 0, self.drive_pwm
        elif self.is_steering_feedback_stale(now):
            self.reset_pid_controller()
            self.get_logger().warn(
                'Steering A1 feedback missing/stale; stopping vehicle',
                throttle_duration_sec=1.0,
            )
            steer_pwm, drive_pwm = 0, 0
        else:
            steer_pwm = self.calculate_steer_pwm(self.pid_dt(now))
            drive_pwm = self.drive_pwm
        self.publish_command(steer_pwm, drive_pwm)

    def pid_dt(self, now):
        nominal_dt = 1.0 / self.command_rate_hz
        if self.last_pid_update_time is None:
            dt = nominal_dt
        else:
            dt = (now - self.last_pid_update_time).nanoseconds / 1e9
            dt = max(0.001, min(dt, nominal_dt * 2.0))
        self.last_pid_update_time = now
        return dt

    def calculate_steer_pwm(self, dt=None):
        if dt is None:
            dt = 1.0 / self.command_rate_hz
        dt = max(0.001, float(dt))
        error = self.target_steer_angle_deg - self.steer_angle_deg
        if abs(error) <= self.steer_angle_tolerance_deg:
            self.pid_integral_error = 0.0
            return 0

        candidate_integral = self.pid_integral_error + error * dt
        if self.steer_pid_ki > 0.0:
            integral_limit = (
                self.steer_pid_integral_limit_pwm / self.steer_pid_ki
            )
            candidate_integral = max(
                -integral_limit,
                min(integral_limit, candidate_integral),
            )
        else:
            candidate_integral = 0.0

        proportional = self.steer_pid_kp * error
        derivative = (
            -self.steer_pid_kd
            * self.steer_angle_velocity_deg_per_sec
        )
        candidate_output = (
            proportional
            + self.steer_pid_ki * candidate_integral
            + derivative
        )

        # 출력 포화 방향으로 적분이 더 쌓이는 경우에는 이번 적분을 버린다.
        if (
            abs(candidate_output) > self.steer_pwm
            and candidate_output * error > 0.0
        ):
            output = (
                proportional
                + self.steer_pid_ki * self.pid_integral_error
                + derivative
            )
        else:
            self.pid_integral_error = candidate_integral
            output = candidate_output

        output = max(-self.steer_pwm, min(self.steer_pwm, output))
        if 0.0 < abs(output) < self.steer_min_pwm:
            output = self.steer_min_pwm if output > 0.0 else -self.steer_min_pwm
        return int(round(output))

    def reset_pid_controller(self):
        self.pid_integral_error = 0.0
        self.last_pid_update_time = None

    def raw_to_steer_angle(self, raw_value):
        raw_value = max(
            self.steer_raw_right,
            min(self.steer_raw_left, int(raw_value)),
        )
        if raw_value >= self.steer_raw_center:
            raw_span = self.steer_raw_left - self.steer_raw_center
            angle = -(
                (raw_value - self.steer_raw_center)
                * self.steer_max_angle_deg
                / raw_span
            )
        else:
            raw_span = self.steer_raw_center - self.steer_raw_right
            angle = (
                (self.steer_raw_center - raw_value)
                * self.steer_max_angle_deg
                / raw_span
            )
        return self.clamp_steer_angle(angle)

    def is_input_stale(self, now=None):
        if self.last_input_time is None:
            return True
        if now is None:
            now = self.get_clock().now()
        age = (now - self.last_input_time).nanoseconds / 1e9
        return age > self.input_timeout

    def is_steering_feedback_stale(self, now=None):
        if self.last_steering_feedback_time is None:
            return True
        if now is None:
            now = self.get_clock().now()
        age = (
            now - self.last_steering_feedback_time
        ).nanoseconds / 1e9
        return age > self.steering_feedback_timeout

    def publish_command(self, steer_pwm, drive_pwm):
        message = Int16MultiArray()
        message.data = [
            max(-255, min(255, int(steer_pwm))),
            self.limit_drive_pwm(drive_pwm),
        ]
        self.command_publisher.publish(message)

    def limit_drive_pwm(self, value):
        return max(-self.max_drive_pwm, min(self.max_drive_pwm, int(value)))

    def clamp_steer_angle(self, value):
        return max(
            -self.steer_max_angle_deg,
            min(self.steer_max_angle_deg, float(value)),
        )

    def destroy_node(self):
        try:
            self.publish_command(0, 0)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DriveControlNode()
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
