"""Publish simple motion targets for testing drive_control_node.

This test never opens the Arduino serial port and never publishes raw motor
PWM. It publishes [target steering angle, drive PWM] to /motor_control so the
unmodified drive_control_node performs the steering conversion.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Int16MultiArray


MOTOR_CONTROL_TOPIC = '/motor_control'
COMMAND_RATE_HZ = 20.0
MAX_STEER_ANGLE_DEG = 45
MAX_DRIVE_PWM = 130


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='drive_control_node를 통한 주행 및 조향 기억 테스트'
    )
    parser.add_argument(
        'motion',
        choices=('straight', 'left', 'right', 'left_sequence'),
        help='시험 동작',
    )
    parser.add_argument(
        '--drive-pwm',
        type=int,
        default=80,
        help='주행 PWM 1~130 (기본값: 80)',
    )
    parser.add_argument(
        '--drive-sec',
        type=float,
        default=1.0,
        help='주행 시간 (기본값: 1.0초)',
    )
    parser.add_argument(
        '--first-straight-sec',
        type=float,
        default=2.0,
        help='left_sequence의 첫 직진 시간 (기본값: 2.0초)',
    )
    parser.add_argument(
        '--turn-sec',
        type=float,
        default=1.0,
        help='left_sequence의 좌회전 시간 (기본값: 1.0초)',
    )
    parser.add_argument(
        '--final-straight-sec',
        type=float,
        default=2.0,
        help='left_sequence의 마지막 직진 시간 (기본값: 2.0초)',
    )
    parser.add_argument(
        '--steer-angle',
        type=int,
        default=30,
        help='좌/우 목표 조향각 1~45도 (기본값: 30)',
    )
    parser.add_argument(
        '--center-sec',
        type=float,
        default=1.0,
        help='시험 전후 중앙 조향 명령 유지 시간 (기본값: 1.0초)',
    )
    parser.add_argument(
        '--countdown-sec',
        type=int,
        default=3,
        help='주행 전 카운트다운 (기본값: 3초)',
    )
    args = parser.parse_args(argv)

    if not 1 <= args.drive_pwm <= MAX_DRIVE_PWM:
        parser.error(f'--drive-pwm은 1~{MAX_DRIVE_PWM} 범위여야 합니다.')
    if not 1 <= args.steer_angle <= MAX_STEER_ANGLE_DEG:
        parser.error(
            f'--steer-angle은 1~{MAX_STEER_ANGLE_DEG} 범위여야 합니다.'
        )
    if args.drive_sec <= 0.0:
        parser.error('--drive-sec는 0보다 커야 합니다.')
    if args.first_straight_sec <= 0.0:
        parser.error('--first-straight-sec는 0보다 커야 합니다.')
    if args.turn_sec <= 0.0:
        parser.error('--turn-sec는 0보다 커야 합니다.')
    if args.final_straight_sec <= 0.0:
        parser.error('--final-straight-sec는 0보다 커야 합니다.')
    if args.center_sec < 0.5:
        parser.error('--center-sec는 안전한 중앙 복귀를 위해 0.5 이상이어야 합니다.')
    if args.countdown_sec < 0:
        parser.error('--countdown-sec는 0 이상이어야 합니다.')
    return args


def target_angle(motion, steer_angle):
    if motion in ('left', 'left_sequence'):
        return -steer_angle
    if motion == 'right':
        return steer_angle
    return 0


class DriveControlMotionTest(Node):
    """Send motion targets to the existing drive_control_node."""

    def __init__(self):
        super().__init__('drive_control_motion_test')
        self.publisher = self.create_publisher(
            Int16MultiArray,
            MOTOR_CONTROL_TOPIC,
            10,
        )

    def publish_target(self, steer_angle_deg, drive_pwm):
        message = Int16MultiArray()
        message.data = [int(steer_angle_deg), int(drive_pwm)]
        self.publisher.publish(message)

    def hold_target(self, steer_angle_deg, drive_pwm, duration_sec):
        deadline = time.monotonic() + duration_sec
        period = 1.0 / COMMAND_RATE_HZ
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_target(steer_angle_deg, drive_pwm)
            remaining = deadline - time.monotonic()
            rclpy.spin_once(
                self,
                timeout_sec=max(0.0, min(period, remaining)),
            )

    def center_and_stop(self, duration_sec):
        self.hold_target(0, 0, duration_sec)
        for _ in range(5):
            self.publish_target(0, 0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_drive_control(self, timeout_sec=3.0):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False


def countdown(seconds):
    for remaining in range(seconds, 0, -1):
        print(f'{remaining}...', flush=True)
        time.sleep(1.0)


def run_test_motion(node, test_args):
    if test_args.motion != 'left_sequence':
        angle = target_angle(test_args.motion, test_args.steer_angle)
        print(
            f'주행 시작: 목표 조향각={angle}도, '
            f'{test_args.drive_sec:.2f}초',
            flush=True,
        )
        node.hold_target(angle, test_args.drive_pwm, test_args.drive_sec)
        return

    print(
        f'1단계 직진: 조향각=0도, {test_args.first_straight_sec:.2f}초',
        flush=True,
    )
    node.hold_target(
        0,
        test_args.drive_pwm,
        test_args.first_straight_sec,
    )

    print(
        f'2단계 좌회전: 조향각=-{test_args.steer_angle}도, '
        f'{test_args.turn_sec:.2f}초',
        flush=True,
    )
    node.hold_target(
        -test_args.steer_angle,
        test_args.drive_pwm,
        test_args.turn_sec,
    )

    print(
        f'3단계 다시 직진: 조향각=0도, '
        f'{test_args.final_straight_sec:.2f}초',
        flush=True,
    )
    node.hold_target(
        0,
        test_args.drive_pwm,
        test_args.final_straight_sec,
    )


def main(args=None):
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    test_args = parse_args(remove_ros_args(args=raw_args)[1:])
    rclpy.init(args=raw_args)
    node = DriveControlMotionTest()
    return_code = 0

    try:
        if not node.wait_for_drive_control():
            raise RuntimeError(
                f'{MOTOR_CONTROL_TOPIC} 구독자가 없습니다. '
                'drive_control_node를 먼저 실행하세요.'
            )

        if test_args.motion == 'left_sequence':
            motion_description = (
                f'직진 {test_args.first_straight_sec:.2f}초'
                f' → 좌회전 {test_args.turn_sec:.2f}초'
                f' → 직진 {test_args.final_straight_sec:.2f}초'
            )
        else:
            angle = target_angle(test_args.motion, test_args.steer_angle)
            motion_description = (
                f'{test_args.motion}, 목표 조향각={angle}도, '
                f'주행 시간={test_args.drive_sec:.2f}초'
            )
        print(
            '\n[안전 확인]\n'
            '- Arduino 통신 노드와 drive_control_node가 실행 중이어야 합니다.\n'
            '- 조이스틱/자율주행 등 다른 /motor_control 발행 노드는 종료하세요.\n'
            f'- 동작={motion_description}, 주행 PWM={test_args.drive_pwm}, '
            f'좌회전 조향각=-{test_args.steer_angle}도\n',
            flush=True,
        )

        print('시험 전 중앙 조향 및 정지', flush=True)
        node.center_and_stop(test_args.center_sec)
        countdown(test_args.countdown_sec)

        run_test_motion(node, test_args)

        print('정지 및 중앙 조향 복귀', flush=True)
        node.center_and_stop(test_args.center_sec)
        print('테스트 완료', flush=True)
    except KeyboardInterrupt:
        print('\n사용자 중단: 정지 및 중앙 복귀 명령을 보냅니다.', flush=True)
    except RuntimeError as exc:
        print(f'오류: {exc}', file=sys.stderr, flush=True)
        return_code = 1
    finally:
        node.center_and_stop(test_args.center_sec)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return return_code


if __name__ == '__main__':
    raise SystemExit(main())
