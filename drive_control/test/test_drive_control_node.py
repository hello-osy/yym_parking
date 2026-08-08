from drive_control.drive_control_node import (
    DriveControlNode,
    STEERING_CALIBRATION_HALF_SPAN,
    STEER_RAW_CENTER,
    STEER_RAW_LEFT,
    STEER_RAW_RIGHT,
)


class FakePublisher:
    def __init__(self):
        self.data = None

    def publish(self, message):
        self.data = list(message.data)


class FakeLogger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


class FakeClock:
    def now(self):
        return object()


def test_pwm_command_is_published_without_serial_access():
    node = object.__new__(DriveControlNode)
    node.max_drive_pwm = 130
    node.command_publisher = FakePublisher()

    node.publish_command(180, 200)

    assert node.command_publisher.data == [180, 130]


def make_closed_loop_node():
    node = object.__new__(DriveControlNode)
    node.steer_raw_left = STEER_RAW_LEFT
    node.steer_raw_center = STEER_RAW_CENTER
    node.steer_raw_right = STEER_RAW_RIGHT
    node.steer_max_angle_deg = 45.0
    node.steer_angle_tolerance_deg = 1.0
    node.steer_pwm = 150
    node.steer_min_pwm = 40
    node.steer_pid_kp = 2.0
    node.steer_pid_ki = 0.0
    node.steer_pid_kd = 0.8
    node.steer_pid_integral_limit_pwm = 30.0
    node.pid_integral_error = 0.0
    node.steer_angle_velocity_deg_per_sec = 0.0
    node.command_rate_hz = 20.0
    node.target_steer_angle_deg = 0.0
    node.steer_angle_deg = 0.0
    return node


def test_raw_feedback_maps_to_calibrated_angles():
    node = make_closed_loop_node()

    assert node.raw_to_steer_angle(STEER_RAW_LEFT) == -45.0
    assert node.raw_to_steer_angle(STEER_RAW_CENTER) == 0.0
    assert node.raw_to_steer_angle(STEER_RAW_RIGHT) == 45.0
    assert node.raw_to_steer_angle(
        (STEER_RAW_LEFT + STEER_RAW_CENTER) // 2
    ) == -22.5
    assert node.raw_to_steer_angle(
        (STEER_RAW_CENTER + STEER_RAW_RIGHT) // 2
    ) == 22.5


def test_positive_error_commands_right_pwm():
    node = make_closed_loop_node()
    node.target_steer_angle_deg = 45.0
    node.steer_angle_deg = -45.0

    assert node.calculate_steer_pwm(0.05) == 150


def test_negative_error_commands_left_pwm():
    node = make_closed_loop_node()
    node.target_steer_angle_deg = -45.0
    node.steer_angle_deg = 45.0

    assert node.calculate_steer_pwm(0.05) == -150


def test_pwm_stops_inside_angle_tolerance():
    node = make_closed_loop_node()
    node.target_steer_angle_deg = 10.0
    node.steer_angle_deg = 9.5

    assert node.calculate_steer_pwm(0.05) == 0


def test_pid_reduces_pwm_near_target():
    node = make_closed_loop_node()
    node.target_steer_angle_deg = 10.0
    node.steer_angle_deg = 0.0

    assert node.calculate_steer_pwm(0.05) == 40


def test_pid_integral_is_bounded_during_saturation():
    node = make_closed_loop_node()
    node.target_steer_angle_deg = 45.0
    node.steer_angle_deg = -45.0

    for _ in range(100):
        assert node.calculate_steer_pwm(0.05) == 150

    assert node.pid_integral_error == 0.0


def test_center_calibration_uses_current_raw_value_and_fixed_span():
    node = make_closed_loop_node()
    node.steer_raw_value = 512
    node.last_pid_update_time = None
    node.calibration_status_publisher = FakePublisher()
    node.get_clock = lambda: FakeClock()
    node.get_logger = lambda: FakeLogger()
    node.is_steering_feedback_stale = lambda _now: False
    request = type('Request', (), {'data': 1})()

    node.center_calibration_callback(request)

    assert node.steer_raw_center == 512
    assert node.steer_raw_left == 512 + STEERING_CALIBRATION_HALF_SPAN
    assert node.steer_raw_right == 512 - STEERING_CALIBRATION_HALF_SPAN
    assert node.calibration_status_publisher.data == [612, 512, 412]
