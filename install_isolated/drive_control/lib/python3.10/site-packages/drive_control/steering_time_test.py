"""Pulse the steering motor for visual travel-time calibration.

Run this together with arduino_communication_node, but do not run
drive_control_node at the same time because both publish to the same command
topic.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Int16MultiArray


COMMAND_TOPIC = '/arduino/motor_command'
COMMAND_RATE_HZ = 20.0
MAX_STEER_PWM = 150


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            '조향 모터를 지정 시간 동안 왕복 구동합니다. '
            'drive_control_node는 반드시 종료한 상태에서 사용하세요.'
        )
    )
    parser.add_argument(
        '--direction',
        choices=('left', 'right'),
        default='right',
        help='처음 움직일 방향 (기본값: right)',
    )
    parser.add_argument(
        '--outbound-sec',
        type=float,
        default=0.2,
        help='처음 방향으로 PWM을 출력할 시간 (기본값: 0.2)',
    )
    parser.add_argument(
        '--return-sec',
        type=float,
        default=None,
        help='반대 방향으로 복귀 PWM을 출력할 시간 (기본값: outbound-sec)',
    )
    parser.add_argument(
        '--pause-sec',
        type=float,
        default=1.0,
        help='최대 조향 위치에서 정지할 시간 (기본값: 1.0)',
    )
    parser.add_argument(
        '--countdown-sec',
        type=int,
        default=3,
        help='작동 전 카운트다운 시간 (기본값: 3)',
    )
    parser.add_argument(
        '--pwm',
        type=int,
        default=MAX_STEER_PWM,
        help='조향 PWM, 1~150 (기본값: 150)',
    )
    args = parser.parse_args(argv)

    if args.outbound_sec <= 0.0:
        parser.error('--outbound-sec는 0보다 커야 합니다.')
    if args.return_sec is None:
        args.return_sec = args.outbound_sec
    if args.return_sec <= 0.0:
        parser.error('--return-sec는 0보다 커야 합니다.')
    if args.pause_sec < 0.0:
        parser.error('--pause-sec는 0 이상이어야 합니다.')
    if args.countdown_sec < 0:
        parser.error('--countdown-sec는 0 이상이어야 합니다.')
    if not 1 <= args.pwm <= MAX_STEER_PWM:
        parser.error(f'--pwm은 1~{MAX_STEER_PWM} 범위여야 합니다.')
    return args


class SteeringTimeTest(Node):
    """Publish steering-only Arduino commands at a watchdog-safe rate."""

    def __init__(self):
        super().__init__('steering_time_test')
        self.publisher = self.create_publisher(
            Int16MultiArray,
            COMMAND_TOPIC,
            10,
        )

    def publish_command(self, steer_pwm):
        message = Int16MultiArray()
        # The second value is drive PWM and must remain zero during this test.
        message.data = [int(steer_pwm), 0]
        self.publisher.publish(message)

    def stop(self):
        # Repeat stop commands so the serial bridge receives one even if a
        # single best-effort scheduling interval is missed.
        for _ in range(5):
            self.publish_command(0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def pulse(self, steer_pwm, duration_sec):
        deadline = time.monotonic() + duration_sec
        period = 1.0 / COMMAND_RATE_HZ
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_command(steer_pwm)
            remaining = deadline - time.monotonic()
            rclpy.spin_once(self, timeout_sec=max(0.0, min(period, remaining)))
        self.stop()

    def wait_for_bridge(self, timeout_sec=3.0):
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


def main(args=None):
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    test_args = parse_args(remove_ros_args(args=raw_args)[1:])
    rclpy.init(args=raw_args)
    node = SteeringTimeTest()
    return_code = 0

    first_sign = 1 if test_args.direction == 'right' else -1
    first_pwm = first_sign * test_args.pwm
    return_pwm = -first_pwm

    try:
        if not node.wait_for_bridge():
            raise RuntimeError(
                f'{COMMAND_TOPIC} 구독자가 없습니다. '
                'arduino_communication_node를 먼저 실행하세요.'
            )

        print(
            '\n[안전 확인]\n'
            '- 차량 바퀴를 지면에서 띄우거나 구동되지 않게 고정하세요.\n'
            '- drive_control_node를 동시에 실행하지 마세요.\n'
            f'- 구동 순서: {test_args.direction} {test_args.outbound_sec:.3f}초'
            f' → 정지 {test_args.pause_sec:.3f}초'
            f' → 복귀 {test_args.return_sec:.3f}초\n'
            f'- 조향 PWM: {test_args.pwm}, 주행 PWM: 0\n',
            flush=True,
        )
        countdown(test_args.countdown_sec)

        print(f'{test_args.direction} 방향 구동 시작', flush=True)
        node.pulse(first_pwm, test_args.outbound_sec)

        print('최대 위치 관찰을 위해 정지', flush=True)
        end_of_pause = time.monotonic() + test_args.pause_sec
        while rclpy.ok() and time.monotonic() < end_of_pause:
            node.publish_command(0)
            rclpy.spin_once(node, timeout_sec=0.05)

        print('반대 방향 복귀 시작', flush=True)
        node.pulse(return_pwm, test_args.return_sec)
        print('테스트 종료: 조향/주행 PWM 0', flush=True)
    except KeyboardInterrupt:
        print('\n사용자 중단: 모터를 정지합니다.', flush=True)
    except RuntimeError as exc:
        print(f'오류: {exc}', file=sys.stderr, flush=True)
        return_code = 1
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return return_code


if __name__ == '__main__':
    raise SystemExit(main())
