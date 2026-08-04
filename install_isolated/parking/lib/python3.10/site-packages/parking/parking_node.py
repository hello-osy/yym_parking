from __future__ import annotations

import math
import time
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, Range
from std_msgs.msg import Bool, Float32MultiArray, Int16MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener

from .camera_assist import CameraAssist, CameraAssistConfig
from .geometry import transform_points_2d
from .lidar_slot_detector import LidarDetectorConfig, LidarSlotDetector
from .lidar_safety import LidarSafetySelfMask, SelfMaskRegion
from .models import SafetyDecision, SlotEstimate, SlotSide
from .parking_fsm import ParkingFSM, ParkingFSMConfig
from .slot_fusion import SlotFusion, SlotFusionConfig
from .ultrasonic_safety import (
    UltrasonicSafety,
    UltrasonicSafetyConfig,
)


class ParkingNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_node")

        self.base_frame = self._p("base_frame", "base_link")
        self.scan_topic = self._p("scan_topic", "/scan")
        self.high_camera_topic = self._p(
            "high_camera_topic", "/camera/high/image_raw"
        )
        self.low_camera_topic = self._p(
            "low_camera_topic", "/camera/low/image_raw"
        )
        self.motor_topic = self._p(
            "motor_topic", "/motor_control"
        )
        self.require_camera_for_motion = bool(
            self._p("require_camera_for_motion", False)
        )

        self.detector = LidarSlotDetector(
            LidarDetectorConfig(
                x_min=float(self._p("lidar.x_min", -2.5)),
                x_max=float(self._p("lidar.x_max", 3.0)),
                lateral_min=float(
                    self._p("lidar.lateral_min", 0.25)
                ),
                lateral_max=float(
                    self._p("lidar.lateral_max", 2.0)
                ),
                max_processing_range=float(
                    self._p(
                        "lidar.max_processing_range", 4.0
                    )
                ),
                bin_size=float(
                    self._p("lidar.bin_size", 0.04)
                ),
                minimum_points_per_bin=int(
                    self._p(
                        "lidar.minimum_points_per_bin", 2
                    )
                ),
                expected_slot_width=float(
                    self._p(
                        "lidar.expected_slot_width", 0.95
                    )
                ),
                minimum_slot_width=float(
                    self._p(
                        "lidar.minimum_slot_width", 0.65
                    )
                ),
                maximum_slot_width=float(
                    self._p(
                        "lidar.maximum_slot_width", 1.25
                    )
                ),
                boundary_band=float(
                    self._p("lidar.boundary_band", 0.12)
                ),
                minimum_confidence=float(
                    self._p(
                        "lidar.minimum_confidence", 0.35
                    )
                ),
            )
        )

        self.fusion = SlotFusion(
            SlotFusionConfig(
                history_size=int(
                    self._p("fusion.history_size", 8)
                ),
                required_count=int(
                    self._p("fusion.required_count", 5)
                ),
                minimum_confidence=float(
                    self._p(
                        "fusion.minimum_confidence", 0.45
                    )
                ),
            )
        )

        self.camera = CameraAssist(
            CameraAssistConfig(
                enable_experimental_slot_hint=bool(
                    self._p(
                        "camera.enable_experimental_slot_hint",
                        False,
                    )
                ),
                out_line_required_frames=int(
                    self._p(
                        "camera.out_line_required_frames", 3
                    )
                ),
            )
        )

        self.ultrasonic = UltrasonicSafety(
            UltrasonicSafetyConfig(
                median_window=int(
                    self._p(
                        "ultrasonic.median_window", 5
                    )
                ),
                stale_timeout=float(
                    self._p(
                        "ultrasonic.stale_timeout", 0.6
                    )
                ),
                slow_distance=float(
                    self._p(
                        "ultrasonic.slow_distance", 0.25
                    )
                ),
                hard_stop_distance=float(
                    self._p(
                        "ultrasonic.hard_stop_distance",
                        0.12,
                    )
                ),
                enable_alignment_steering_bias=bool(
                    self._p(
                        "ultrasonic.enable_alignment_steering_bias",
                        False,
                    )
                ),
            )
        )

        self.fsm = ParkingFSM(
            ParkingFSMConfig(
                approach_speed=int(
                    self._p(
                        "control.approach_speed", 22
                    )
                ),
                reverse_arc_speed=int(
                    self._p(
                        "control.reverse_arc_speed", -18
                    )
                ),
                align_speed=int(
                    self._p("control.align_speed", -14)
                ),
                final_reverse_speed=int(
                    self._p(
                        "control.final_reverse_speed", -12
                    )
                ),
                exit_speed=int(
                    self._p("control.exit_speed", 20)
                ),
                left_slot_steer=int(
                    self._p(
                        "control.left_slot_steer", 45
                    )
                ),
                right_slot_steer=int(
                    self._p(
                        "control.right_slot_steer", -45
                    )
                ),
                staging_pass_distance=float(
                    self._p(
                        "control.staging_pass_distance",
                        0.20,
                    )
                ),
                target_depth=float(
                    self._p(
                        "control.target_depth", 0.90
                    )
                ),
                target_rear_stop_distance=float(
                    self._p(
                        "control.target_rear_stop_distance",
                        0.14,
                    )
                ),
                counter_trigger_deg=float(
                    self._p(
                        "control.counter_trigger_deg", 35.0
                    )
                ),
                align_yaw_tolerance_deg=float(
                    self._p(
                        "control.align_yaw_tolerance_deg",
                        8.0,
                    )
                ),
                final_yaw_tolerance_deg=float(
                    self._p(
                        "control.final_yaw_tolerance_deg",
                        5.0,
                    )
                ),
                final_lateral_tolerance=float(
                    self._p(
                        "control.final_lateral_tolerance",
                        0.08,
                    )
                ),
                hold_time=float(
                    self._p("control.hold_time", 4.0)
                ),
                exit_straight_time=float(
                    self._p(
                        "control.exit_straight_time", 1.0
                    )
                ),
                exit_turn_steer=int(
                    self._p(
                        "control.exit_turn_steer", 0
                    )
                ),
            )
        )

        # This mask is intentionally used only by the left-side safety-sector
        # query below.  Slot detection always receives the unmasked scan.
        self.left_side_sector = (
            float(self._p("lidar.left_side_x_min", -0.75)),
            float(self._p("lidar.left_side_x_max", 0.60)),
            float(self._p("lidar.left_side_y_min", 0.10)),
            float(self._p("lidar.left_side_y_max", 1.25)),
        )
        self.left_side_hard_stop_distance = float(
            self._p("lidar.left_side_hard_stop_distance", 0.20)
        )
        self.lidar_self_mask = LidarSafetySelfMask(
            bool(self._p("lidar_safety.self_mask.enabled", True)),
            (
                SelfMaskRegion(
                    name=str(
                        self._p(
                            "lidar_safety.self_mask.region.name",
                            "lidar_front_left_self_return",
                        )
                    ),
                    x_min=float(
                        self._p(
                            "lidar_safety.self_mask.region.x_min", 0.04
                        )
                    ),
                    x_max=float(
                        self._p(
                            "lidar_safety.self_mask.region.x_max", 0.13
                        )
                    ),
                    y_min=float(
                        self._p(
                            "lidar_safety.self_mask.region.y_min", 0.11
                        )
                    ),
                    y_max=float(
                        self._p(
                            "lidar_safety.self_mask.region.y_max", 0.18
                        )
                    ),
                    reason=str(
                        self._p(
                            "lidar_safety.self_mask.region.reason",
                            "persistent replay self return near x=0.085 y=0.145",
                        )
                    ),
                ),
            ),
        )
        self.left_side_lidar_minimum_before_mask = math.inf
        self.left_side_lidar_minimum_after_mask = math.inf
        self.left_side_lidar_minimum_point_after_mask = None
        self.lidar_self_mask_filtered_count = 0
        self.lidar_self_mask_matched_regions: tuple[str, ...] = ()

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        self.last_lidar_at = -math.inf
        self.last_high_camera_at = -math.inf
        self.last_low_camera_at = -math.inf
        self.lidar_timeout = float(
            self._p("timeouts.lidar", 0.5)
        )
        self.camera_timeout = float(
            self._p("timeouts.camera", 0.8)
        )
        self.out_line_detected = False
        self.last_command_speed = 0
        self.latest_slot: Optional[SlotEstimate] = None

        self.motor_pub = self.create_publisher(
            Int16MultiArray, self.motor_topic, 10
        )
        self.state_pub = self.create_publisher(
            String, "/parking/state", 10
        )
        self.reason_pub = self.create_publisher(
            String, "/parking/reason", 10
        )
        self.clearance_pub = self.create_publisher(
            Float32MultiArray,
            "/parking/clearances",
            10,
        )
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            "/parking/target_pose",
            10,
        )

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.high_camera_topic,
            self._on_high_camera,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.low_camera_topic,
            self._on_low_camera,
            qos_profile_sensor_data,
        )
        for index in range(1, 7):
            self.create_subscription(
                Range,
                f"/ultrasonic/range_{index}",
                lambda msg, i=index: self._on_ultrasonic(
                    i, msg
                ),
                qos_profile_sensor_data,
            )

        self.create_subscription(
            Bool,
            "/parking/start",
            self._on_start,
            10,
        )
        self.create_subscription(
            Bool,
            "/parking/reset",
            self._on_reset,
            10,
        )

        rate_hz = float(
            self._p("control.rate_hz", 10.0)
        )
        self.create_timer(
            1.0 / max(rate_hz, 1.0),
            self._control_tick,
        )

        if bool(self._p("auto_start", False)):
            self.fsm.start(time.monotonic())

        self.get_logger().info(
            "Parking node ready: LiDAR primary, camera assist, "
            "ultrasonic safety."
        )

    def _on_scan(self, msg: LaserScan) -> None:
        now = time.monotonic()
        points = self.detector.scan_to_points(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
        )
        if points.size == 0:
            return

        if msg.header.frame_id != self.base_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    msg.header.frame_id,
                    rclpy.time.Time(),
                )
            except TransformException as error:
                self.get_logger().warning(
                    f"LiDAR TF unavailable: {error}",
                    throttle_duration_sec=2.0,
                )
                return

            t = transform.transform.translation
            q = transform.transform.rotation
            yaw = self._yaw_from_quaternion(
                q.x, q.y, q.z, q.w
            )
            points = transform_points_2d(
                points, t.x, t.y, yaw
            )

        # Do not mask detector input: the self-return is only excluded while
        # finding a safety-sector minimum.
        self._update_left_side_lidar_safety(points)
        estimate = self.detector.detect(points, now)
        self.fusion.update_lidar(estimate)
        self.last_lidar_at = now

    def _on_high_camera(self, msg: Image) -> None:
        self.last_high_camera_at = time.monotonic()
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
            self.fusion.update_camera(
                self.camera.detect_slot_hint(image)
            )
        except Exception as error:
            self.get_logger().warning(
                f"High camera conversion failed: {error}",
                throttle_duration_sec=2.0,
            )

    def _on_low_camera(self, msg: Image) -> None:
        self.last_low_camera_at = time.monotonic()
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
            (
                self.out_line_detected,
                _,
            ) = self.camera.detect_out_line(image)
        except Exception as error:
            self.get_logger().warning(
                f"Low camera conversion failed: {error}",
                throttle_duration_sec=2.0,
            )

    def _on_ultrasonic(
        self,
        index: int,
        msg: Range,
    ) -> None:
        self.ultrasonic.update(
            index, float(msg.range), time.monotonic()
        )

    def _on_start(self, msg: Bool) -> None:
        if msg.data:
            self.fusion.reset()
            self.latest_slot = None
            self.fsm.start(time.monotonic())

    def _on_reset(self, msg: Bool) -> None:
        if msg.data:
            self.fusion.reset()
            self.latest_slot = None
            self.fsm.reset(time.monotonic())

    def _control_tick(self) -> None:
        now = time.monotonic()
        stable_slot = self.fusion.stable_estimate(now)
        if stable_slot is not None:
            self.latest_slot = stable_slot
        slot = stable_slot or self.latest_slot

        side = (
            slot.side
            if slot is not None
            else SlotSide.UNKNOWN
        )
        safety = self.ultrasonic.assess(
            self.last_command_speed,
            self.fsm.state,
            side,
        )
        safety = self._apply_left_side_lidar_hard_stop(safety)

        lidar_fresh = (
            now - self.last_lidar_at <= self.lidar_timeout
        )
        ultrasonic_fresh = self.ultrasonic.all_fresh(now)
        camera_fresh = (
            now - max(
                self.last_high_camera_at,
                self.last_low_camera_at,
            )
            <= self.camera_timeout
        )
        required_fresh = (
            lidar_fresh
            and ultrasonic_fresh
            and (
                camera_fresh
                if self.require_camera_for_motion
                else True
            )
        )

        command = self.fsm.step(
            now,
            slot,
            safety,
            self.out_line_detected,
            required_fresh,
            self.ultrasonic.rear_minimum(),
        )
        self.last_command_speed = command.speed

        motor = Int16MultiArray()
        motor.data = [command.steer, command.speed]
        self.motor_pub.publish(motor)

        state = String()
        state.data = self.fsm.state.value
        self.state_pub.publish(state)

        reason = String()
        reason.data = self._reason_with_lidar_safety(command.reason)
        self.reason_pub.publish(reason)

        clearances = Float32MultiArray()
        clearances.data = self.ultrasonic.all_filtered()
        self.clearance_pub.publish(clearances)

        if slot is not None:
            self._publish_target_pose(slot)

    def _update_left_side_lidar_safety(self, points: np.ndarray) -> None:
        result = self.lidar_self_mask.minimum_in_sector(
            points, self.left_side_sector
        )
        self.left_side_lidar_minimum_before_mask = result.before_mask.distance
        self.left_side_lidar_minimum_after_mask = result.after_mask.distance
        self.left_side_lidar_minimum_point_after_mask = result.after_mask.point
        self.lidar_self_mask_filtered_count = result.filtered_count
        self.lidar_self_mask_matched_regions = result.matched_regions

    def _apply_left_side_lidar_hard_stop(
        self, ultrasonic_safety: SafetyDecision
    ) -> SafetyDecision:
        """LiDAR safety takes precedence; the FSM performs the safe stop."""
        if (
            math.isfinite(self.left_side_lidar_minimum_after_mask)
            and self.left_side_lidar_minimum_after_mask
            < self.left_side_hard_stop_distance
        ):
            return SafetyDecision(
                hard_stop=True,
                speed_scale=0.0,
                steering_bias=0.0,
                reason="LIDAR_HARD_STOP:left_side",
                minimum_active_distance=(
                    self.left_side_lidar_minimum_after_mask
                ),
            )
        return ultrasonic_safety

    def _reason_with_lidar_safety(self, primary_reason: str) -> str:
        def format_distance(distance: float) -> str:
            return "inf" if not math.isfinite(distance) else f"{distance:.3f}"

        point = self.left_side_lidar_minimum_point_after_mask
        point_text = (
            "none"
            if point is None
            else f"({point[0]:.3f},{point[1]:.3f})"
        )
        diagnostics = [
            "lidar_self_mask_enabled="
            + str(self.lidar_self_mask.enabled).lower(),
            "lidar_self_mask_filtered_count="
            + str(self.lidar_self_mask_filtered_count),
            "left_side_min_before_mask="
            + format_distance(self.left_side_lidar_minimum_before_mask),
            "left_side_min_after_mask="
            + format_distance(self.left_side_lidar_minimum_after_mask),
            "left_side_min_point_after_mask=" + point_text,
        ]
        diagnostics.extend(
            f"LIDAR_SELF_RETURN_MASKED:{name}"
            for name in self.lidar_self_mask_matched_regions
        )
        return " | ".join((primary_reason, *diagnostics))

    def _publish_target_pose(
        self,
        slot: SlotEstimate,
    ) -> None:
        x, y = slot.target_point(
            self.fsm.config.target_depth
        )
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)

        half = slot.inward_yaw * 0.5
        pose.pose.orientation.z = math.sin(half)
        pose.pose.orientation.w = math.cos(half)
        self.target_pose_pub.publish(pose)

    @staticmethod
    def _yaw_from_quaternion(
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> float:
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(sin_yaw, cos_yaw)

    def _p(self, name: str, default):
        return self.declare_parameter(
            name, default
        ).value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Int16MultiArray()
        stop.data = [0, 0]
        node.motor_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
