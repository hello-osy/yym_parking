"""YYM LiDAR controller for reverse perpendicular parking on the right.

Scan convention used by this vehicle:
    rear=0 deg, right=+90 deg, front=+/-180 deg, left=-90 deg.

The controller holds still for a configurable startup delay, then approaches
at an explicit 0-degree steering target and, when the first parked vehicle is
confirmed, performs a calibrated left turn. LiDAR
then confirms that both parked vehicles are visible and performs one-second
midpoint-angle corrections with gentle parked-vehicle edge alignment while
reversing. Once either parked vehicle enters the 1 m ring, the controller stops
for five seconds. It then performs two LiDAR corrections of 0.5 seconds each,
stops to center steering, and reverses straight until the unchanged LiDAR
parking-completion condition is reached.
Parking completes when either original parked vehicle, not a later pillar or
unit, disappears below the rear-mounted LiDAR's horizontal x=0 line. The
controller waits while stopped, then runs the configured timed exit sequence.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import os
import signal
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int16MultiArray


class ParkingState(str, Enum):
    WAIT_FOR_SCAN = 'WAIT_FOR_SCAN'
    START_DELAY = 'START_DELAY'
    APPROACH_FIRST_CAR = 'APPROACH_FIRST_CAR'
    SET_LEFT_STEER = 'SET_LEFT_STEER'
    TURN_LEFT_TIMED = 'TURN_LEFT_TIMED'
    RECOGNITION_COMPLETE = 'RECOGNITION_COMPLETE'
    SETTLE_AND_ACQUIRE_GAP = 'SETTLE_AND_ACQUIRE_GAP'
    REVERSE_CENTER = 'REVERSE_CENTER'
    PARKED = 'PARKED'
    EXIT_FORWARD = 'EXIT_FORWARD'
    EXIT_SET_RIGHT_STEER = 'EXIT_SET_RIGHT_STEER'
    EXIT_RIGHT_TURN = 'EXIT_RIGHT_TURN'
    EXIT_CENTER_STEER = 'EXIT_CENTER_STEER'
    EXIT_FINAL_FORWARD = 'EXIT_FINAL_FORWARD'
    EXIT_COMPLETE = 'EXIT_COMPLETE'
    PARKING_FAILED = 'PARKING_FAILED'
    EMERGENCY_STOP = 'EMERGENCY_STOP'


class ParkingMode(str, Enum):
    RECOGNITION = 'RECOGNITION'
    PARKING = 'PARKING'
    EXIT = 'EXIT'


@dataclass
class VehicleCluster:
    points: np.ndarray
    center: np.ndarray
    axis_angle: float
    x_min: float
    x_max: float
    y_min: float
    y_max: float


@dataclass
class ParkingPair:
    lower: VehicleCluster
    upper: VehicleCluster
    reference_point: np.ndarray
    gap_center_y: float
    gap_width: float
    left_clearance: float
    right_clearance: float


@dataclass
class LidarObservation:
    scan_valid: bool
    points: np.ndarray
    vehicles: list[VehicleCluster]
    right_vehicles: list[VehicleCluster]
    pair: Optional[ParkingPair]
    rear_min_distance: Optional[float]
    pair_is_fallback: bool = False


@dataclass
class ParkingLineObservation:
    """High-camera features used only during precision reverse."""

    stamp: float
    horizontal_line: tuple[float, float, float, float]
    vertical_line: tuple[float, float, float, float]
    intersection: tuple[float, float]
    horizontal_y_ratio: float
    horizontal_angle_deg: float
    vertical_angle_deg: float
    position_error_ratio: float
    steering_deg: int


class ParkingNodeYym(Node):
    """LiDAR-feedback parking into random right-side slot 2 or 3."""

    def __init__(self) -> None:
        super().__init__('parking_node_yym')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('motor_topic', '/motor_control')
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('scan_timeout_sec', 0.5)
        self.declare_parameter('scan_quality_min_points', 10)
        self.declare_parameter('invalid_scan_confirm_frames', 5)
        self.declare_parameter('startup_delay_sec', 5.0)
        # start_mode=recognition runs the normal complete sequence.
        # start_mode=parking skips recognition for debugging from an already
        # stationary pose after the timed left turn. recognition_only latches
        # a stop as soon as that recognition turn is complete.
        self.declare_parameter('start_mode', 'recognition')
        self.declare_parameter('recognition_only', False)

        self.declare_parameter('debug_view', True)
        self.declare_parameter('debug_window_name', 'parking_yym_debug')
        self.declare_parameter('debug_hz', 20.0)
        self.declare_parameter('debug_max_range_m', 4.0)
        self.declare_parameter('debug_image_size', 820)

        # Only the rear and side field is relevant after the left entry turn.
        self.declare_parameter('valid_sector_max_abs_deg', 125.0)
        self.declare_parameter('parking_min_range_m', 0.15)
        self.declare_parameter('cluster_max_range_m', 4.0)
        # Allow slightly farther parked vehicles only while acquiring the
        # initial two-vehicle gap before the first reverse segment.
        self.declare_parameter('initial_gap_cluster_max_range_m', 6.0)
        self.declare_parameter('cluster_neighbor_distance_m', 0.20)
        self.declare_parameter('cluster_min_points', 7)
        self.declare_parameter('obstacle_min_extent_m', 0.22)
        self.declare_parameter('right_detection_margin_m', 0.12)
        # Before the 1 m/five-second latch, only clusters on or below the
        # green horizontal x=0 line can represent the two parked vehicles.
        self.declare_parameter('pre_final_vehicle_max_x_m', 0.0)

        self.declare_parameter('first_car_confirm_frames', 2)
        self.declare_parameter('first_car_min_points', 7)
        self.declare_parameter('first_car_min_extent_m', 0.22)
        # Recognition only: distant wall/noise clusters must not trigger the
        # timed left turn. Parking-mode clustering keeps its full 4 m range.
        self.declare_parameter('first_car_max_distance_m', 2.0)
        # During the initial straight approach, ignore clusters whose center
        # lies behind the LiDAR/green horizontal x=0 line.
        self.declare_parameter('first_car_min_center_x_m', 0.10)
        # Apply the same boundary to individual points before clustering so
        # rear noise cannot merge with front returns and shift the center.
        self.declare_parameter('recognition_vehicle_min_x_m', 0.05)
        self.declare_parameter('gap_confirm_frames', 3)
        self.declare_parameter('gap_min_width_m', 0.48)
        self.declare_parameter('gap_max_width_m', 1.40)
        self.declare_parameter('gap_track_max_center_m', 0.85)

        # Recognition-mode test speed requested for the real vehicle.
        self.declare_parameter('approach_speed', 110)
        self.declare_parameter('turn_speed', 110)
        self.declare_parameter('reverse_speed', -110)
        self.declare_parameter('left_max_steer_deg', -45)
        # Legacy +/-90-degree parameters remain available only for visual
        # diagnostics. They do not control the simplified parking sequence.
        self.declare_parameter('lidar_side_gate_half_width_deg', 15.0)
        self.declare_parameter('lidar_side_gate_min_points', 3)
        self.declare_parameter('lidar_side_gate_confirm_frames', 3)
        self.declare_parameter('lidar_side_far_distance_m', 2.0)
        self.declare_parameter('lidar_side_far_half_width_deg', 5.0)
        self.declare_parameter('lidar_side_far_confirm_frames', 3)
        self.declare_parameter('lidar_side_far_stop_sec', 4.0)
        self.declare_parameter('exit_wait_after_park_sec', 2.0)
        self.declare_parameter('exit_forward_duration_sec', 3.0)
        self.declare_parameter('exit_right_turn_duration_sec', 10.0)
        self.declare_parameter('exit_final_forward_duration_sec', 10.0)
        self.declare_parameter('exit_right_steer_deg', 45)
        self.declare_parameter('exit_speed', 110)
        # Real-vehicle testing showed the raw geometric angle was too weak.
        self.declare_parameter('reverse_steer_multiplier', 10.0)
        # Before the 1 m latch, preserve strong midpoint centering and add a
        # gentler gap-facing longitudinal-edge alignment correction.
        self.declare_parameter(
            'pre_final_alignment_steer_multiplier', 5.0
        )
        # Use gentler corrections after the 1 m trigger and five-second stop.
        self.declare_parameter('final_reverse_steer_multiplier', 5.0)
        self.declare_parameter('final_line_alignment_tolerance_deg', 3.0)
        # A cardboard-box vehicle often appears as an L shape. Estimate its
        # longitudinal tilt from the gap-facing edge and reject short cross
        # faces that PCA can incorrectly report as an 80-90 degree tilt.
        self.declare_parameter('final_edge_min_x_span_m', 0.20)
        self.declare_parameter('final_edge_bin_count', 7)
        self.declare_parameter('final_edge_min_bin_count', 3)
        self.declare_parameter('final_edge_max_abs_angle_deg', 45.0)
        self.declare_parameter('final_correction_duration_sec', 0.5)
        self.declare_parameter('final_correction_segment_count', 2)
        self.declare_parameter('steer_settle_sec', 0.6)
        # Recognition test: after steering settles at -45 degrees, drive for
        # this calibrated duration with maximum left steering, then stop.
        self.declare_parameter('left_turn_duration_sec', 7.0)
        # Keep publishing stop briefly before shutting down the launch.
        self.declare_parameter('recognition_shutdown_delay_sec', 0.5)
        self.declare_parameter('approach_timeout_sec', 30.0)
        self.declare_parameter('gap_acquire_timeout_sec', 4.0)
        # Zero disables the elapsed-time stop. Parking depth is determined by
        # the rear-half LiDAR point condition below, not a guessed duration.
        self.declare_parameter('reverse_timeout_sec', 0.0)
        # Reverse for one second, then stop and recompute the LiDAR-based
        # center/steering target before the next segment.
        self.declare_parameter('reverse_segment_duration_sec', 1.0)
        self.declare_parameter('reverse_measure_stop_sec', 0.4)
        self.declare_parameter('vehicle_pair_track_max_jump_m', 1.25)
        # Retained for compatibility with older launch parameter files and
        # the unused single-vehicle fallback helper. The tighter tracker
        # prevents a pillar/next unit from replacing an original vehicle.
        self.declare_parameter('final_center_gain', 1.0)
        self.declare_parameter('final_alignment_gain', 1.0)
        self.declare_parameter('final_vehicle_track_max_jump_m', 0.25)
        # Green horizontal line is x=0 at the rear-mounted LiDAR. Stop with
        # centered steering on the first scan with no valid points below it,
        # then confirm while stationary to reject a one-frame dropout.
        self.declare_parameter('rear_half_stop_margin_m', 0.0)
        self.declare_parameter('rear_half_empty_confirm_frames', 3)
        # The second debug range ring is 1.0 m. Once any detected vehicle
        # contributes a point inside it, stop for five seconds at 0 deg and
        # then reverse straight.
        self.declare_parameter('straight_reverse_radius_m', 1.0)
        self.declare_parameter('straight_reverse_stop_sec', 5.0)
        # With only one parked vehicle visible, its PCA line replaces the
        # unavailable two-vehicle midpoint. A vertical debug line is 0 deg.
        self.declare_parameter('single_vehicle_angle_deadband_deg', 2.0)

        # Precision reverse starts only after the five-second stop. The slot
        # number is deliberately selected by the operator because this course
        # has exactly two valid target appearances. Reference coordinates are
        # normalized raw high-camera image coordinates, not screenshot pixels.
        try:
            default_parking_slot = int(
                os.environ.get('PARKING_SLOT_NUMBER', '2')
            )
        except ValueError:
            default_parking_slot = 2
        self.declare_parameter(
            'parking_slot_number', default_parking_slot
        )
        self.declare_parameter(
            'high_camera_topic', '/camera/high/image_raw'
        )
        self.declare_parameter('camera_line_timeout_sec', 0.5)
        self.declare_parameter('camera_line_confirm_frames', 3)
        self.declare_parameter('camera_line_loss_confirm_frames', 3)
        self.declare_parameter('camera_line_white_threshold', 175)
        self.declare_parameter('camera_line_max_saturation', 85)
        self.declare_parameter('camera_horizontal_max_angle_deg', 18.0)
        self.declare_parameter('camera_vertical_max_angle_deg', 50.0)
        self.declare_parameter('camera_horizontal_min_length_ratio', 0.18)
        self.declare_parameter('camera_vertical_min_length_ratio', 0.05)
        self.declare_parameter('camera_feature_ema_alpha', 0.35)
        # Reference features measured from the supplied perfectly parked
        # views. Keep these parameters exposed for calibration from raw frames.
        self.declare_parameter('slot2_horizontal_y_ratio', 0.052)
        self.declare_parameter('slot2_horizontal_angle_deg', 0.0)
        self.declare_parameter('slot2_vertical_x_ratio', 0.500)
        self.declare_parameter('slot2_vertical_angle_deg', 0.0)
        self.declare_parameter('slot3_horizontal_y_ratio', 0.055)
        self.declare_parameter('slot3_horizontal_angle_deg', 0.0)
        self.declare_parameter('slot3_corner_x_ratio', 0.693)
        self.declare_parameter('slot3_vertical_angle_deg', -30.0)
        self.declare_parameter('camera_position_gain_deg', 160.0)
        self.declare_parameter(
            'camera_vertical_reference_tolerance_deg', 25.0
        )
        self.declare_parameter(
            'camera_horizontal_reference_tolerance_deg', 12.0
        )
        # Rear-view point error and steering are opposite during reverse:
        # detected point left of target -> right steer, and vice versa.
        self.declare_parameter('camera_steer_sign', -1.0)
        self.declare_parameter('camera_max_steer_deg', 35)
        self.declare_parameter('camera_steer_deadband_deg', 1.0)
        self.declare_parameter(
            'camera_debug_window_name', 'parking_yym_high_camera'
        )

        self.declare_parameter('rear_hard_stop_angle_deg', 11.0)
        self.declare_parameter('rear_hard_stop_distance_m', 0.18)
        self.declare_parameter('vehicle_width_m', 0.38)
        self.declare_parameter('minimum_side_clearance_m', 0.05)
        # LiDAR is mounted at the vehicle rear end in this vehicle.
        self.declare_parameter('lidar_to_rear_bumper_m', 0.0)

        self.scan_topic = str(self._value('scan_topic'))
        self.motor_topic = str(self._value('motor_topic'))
        self.control_hz = max(1.0, float(self._value('control_hz')))
        self.scan_timeout = max(0.05, float(self._value('scan_timeout_sec')))
        self.scan_quality_min_points = int(self._value('scan_quality_min_points'))
        self.invalid_scan_confirm_frames = int(
            self._value('invalid_scan_confirm_frames')
        )
        self.startup_delay = max(
            0.0, float(self._value('startup_delay_sec'))
        )
        requested_start_mode = str(self._value('start_mode')).strip().lower()
        if requested_start_mode not in ('recognition', 'parking'):
            self.get_logger().warn(
                f'Unknown start_mode={requested_start_mode!r}; '
                'using recognition'
            )
            requested_start_mode = 'recognition'
        self.start_mode = (
            ParkingMode.PARKING
            if requested_start_mode == 'parking'
            else ParkingMode.RECOGNITION
        )
        self.recognition_only = bool(self._value('recognition_only'))

        self.debug_view = bool(self._value('debug_view'))
        self.debug_window_name = str(self._value('debug_window_name'))
        self.debug_hz = max(1.0, float(self._value('debug_hz')))
        self.debug_max_range = max(0.5, float(self._value('debug_max_range_m')))
        self.debug_image_size = max(500, int(self._value('debug_image_size')))

        self.valid_sector_max_abs = math.radians(
            float(self._value('valid_sector_max_abs_deg'))
        )
        self.parking_min_range = max(
            0.0, float(self._value('parking_min_range_m'))
        )
        self.cluster_max_range = max(
            self.parking_min_range,
            float(self._value('cluster_max_range_m')),
        )
        self.initial_gap_cluster_max_range = max(
            self.cluster_max_range,
            float(self._value('initial_gap_cluster_max_range_m')),
        )
        self.cluster_neighbor_distance = max(
            0.02, float(self._value('cluster_neighbor_distance_m'))
        )
        self.cluster_min_points = max(2, int(self._value('cluster_min_points')))
        self.obstacle_min_extent = max(
            0.05, float(self._value('obstacle_min_extent_m'))
        )
        self.right_detection_margin = max(
            0.0, float(self._value('right_detection_margin_m'))
        )
        self.pre_final_vehicle_max_x = float(
            self._value('pre_final_vehicle_max_x_m')
        )

        self.first_car_confirm_frames = max(
            1, int(self._value('first_car_confirm_frames'))
        )
        self.first_car_min_points = max(
            self.cluster_min_points,
            int(self._value('first_car_min_points')),
        )
        self.first_car_min_extent = max(
            self.obstacle_min_extent,
            float(self._value('first_car_min_extent_m')),
        )
        self.first_car_max_distance = max(
            self.parking_min_range,
            float(self._value('first_car_max_distance_m')),
        )
        self.first_car_min_center_x = float(
            self._value('first_car_min_center_x_m')
        )
        self.recognition_vehicle_min_x = float(
            self._value('recognition_vehicle_min_x_m')
        )
        self.gap_confirm_frames = max(1, int(self._value('gap_confirm_frames')))
        self.gap_min_width = float(self._value('gap_min_width_m'))
        self.gap_max_width = float(self._value('gap_max_width_m'))
        self.gap_track_max_center = float(
            self._value('gap_track_max_center_m')
        )

        self.approach_speed = int(self._value('approach_speed'))
        self.turn_speed = int(self._value('turn_speed'))
        self.reverse_speed = -abs(int(self._value('reverse_speed')))
        self.left_max_steer = -abs(int(self._value('left_max_steer_deg')))
        self.lidar_side_gate_half_width = math.radians(max(
            1.0, float(self._value('lidar_side_gate_half_width_deg'))
        ))
        self.lidar_side_gate_min_points = max(
            1, int(self._value('lidar_side_gate_min_points'))
        )
        self.lidar_side_gate_confirm_frames = max(
            1, int(self._value('lidar_side_gate_confirm_frames'))
        )
        self.lidar_side_far_distance = max(
            0.1, float(self._value('lidar_side_far_distance_m'))
        )
        self.lidar_side_far_half_width = math.radians(max(
            1.0, float(self._value('lidar_side_far_half_width_deg'))
        ))
        self.lidar_side_far_confirm_frames = max(
            1, int(self._value('lidar_side_far_confirm_frames'))
        )
        self.lidar_side_far_stop = max(
            0.0, float(self._value('lidar_side_far_stop_sec'))
        )
        self.exit_wait_after_park = max(
            0.0, float(self._value('exit_wait_after_park_sec'))
        )
        self.exit_forward_duration = max(
            0.0, float(self._value('exit_forward_duration_sec'))
        )
        self.exit_right_turn_duration = max(
            0.0, float(self._value('exit_right_turn_duration_sec'))
        )
        self.exit_final_forward_duration = max(
            0.0, float(self._value('exit_final_forward_duration_sec'))
        )
        self.exit_right_steer = abs(
            int(self._value('exit_right_steer_deg'))
        )
        self.exit_speed = abs(int(self._value('exit_speed')))
        self.reverse_steer_multiplier = max(
            0.0, float(self._value('reverse_steer_multiplier'))
        )
        self.pre_final_alignment_steer_multiplier = max(
            0.0,
            float(self._value(
                'pre_final_alignment_steer_multiplier'
            )),
        )
        self.final_reverse_steer_multiplier = max(
            0.0, float(self._value('final_reverse_steer_multiplier'))
        )
        self.final_line_alignment_tolerance = max(
            0.0,
            float(self._value('final_line_alignment_tolerance_deg')),
        )
        self.final_edge_min_x_span = max(
            0.05, float(self._value('final_edge_min_x_span_m'))
        )
        self.final_edge_bin_count = max(
            3, int(self._value('final_edge_bin_count'))
        )
        self.final_edge_min_bin_count = max(
            2, int(self._value('final_edge_min_bin_count'))
        )
        self.final_edge_max_abs_angle = max(
            1.0, float(self._value('final_edge_max_abs_angle_deg'))
        )
        self.final_correction_duration = max(
            0.1, float(self._value('final_correction_duration_sec'))
        )
        self.final_correction_segment_count = max(
            1, int(self._value('final_correction_segment_count'))
        )
        self.steer_settle_sec = float(self._value('steer_settle_sec'))
        self.left_turn_duration_sec = max(
            0.1, float(self._value('left_turn_duration_sec'))
        )
        self.recognition_shutdown_delay_sec = max(
            0.1, float(self._value('recognition_shutdown_delay_sec'))
        )
        self.approach_timeout = float(self._value('approach_timeout_sec'))
        self.gap_acquire_timeout = float(
            self._value('gap_acquire_timeout_sec')
        )
        self.reverse_timeout = max(
            0.0, float(self._value('reverse_timeout_sec'))
        )
        self.reverse_segment_duration = max(
            0.1, float(self._value('reverse_segment_duration_sec'))
        )
        self.reverse_measure_stop = max(
            0.1, float(self._value('reverse_measure_stop_sec'))
        )
        self.vehicle_pair_track_max_jump = max(
            0.1, float(self._value('vehicle_pair_track_max_jump_m'))
        )
        self.final_center_gain = max(
            0.0, float(self._value('final_center_gain'))
        )
        self.final_alignment_gain = max(
            0.0, float(self._value('final_alignment_gain'))
        )
        self.final_vehicle_track_max_jump = max(
            0.05, float(self._value('final_vehicle_track_max_jump_m'))
        )
        self.rear_half_stop_margin = max(
            0.0, float(self._value('rear_half_stop_margin_m'))
        )
        self.rear_half_empty_confirm_frames = max(
            1, int(self._value('rear_half_empty_confirm_frames'))
        )
        self.straight_reverse_radius = max(
            0.1, float(self._value('straight_reverse_radius_m'))
        )
        self.straight_reverse_stop = max(
            0.0, float(self._value('straight_reverse_stop_sec'))
        )
        self.single_vehicle_angle_deadband = max(
            0.0, float(self._value('single_vehicle_angle_deadband_deg'))
        )

        requested_slot = int(self._value('parking_slot_number'))
        if requested_slot not in (2, 3):
            self.get_logger().warning(
                f'parking_slot_number={requested_slot} is invalid; using 2'
            )
            requested_slot = 2
        self.parking_slot_number = requested_slot
        self.high_camera_topic = str(self._value('high_camera_topic'))
        self.camera_line_timeout = max(
            0.05, float(self._value('camera_line_timeout_sec'))
        )
        self.camera_line_confirm_frames = max(
            1, int(self._value('camera_line_confirm_frames'))
        )
        self.camera_line_loss_confirm_frames = max(
            1, int(self._value('camera_line_loss_confirm_frames'))
        )
        self.camera_line_white_threshold = int(np.clip(
            int(self._value('camera_line_white_threshold')), 0, 255
        ))
        self.camera_line_max_saturation = int(np.clip(
            int(self._value('camera_line_max_saturation')), 0, 255
        ))
        self.camera_horizontal_max_angle = max(
            1.0, float(self._value('camera_horizontal_max_angle_deg'))
        )
        self.camera_vertical_max_angle = max(
            10.0, float(self._value('camera_vertical_max_angle_deg'))
        )
        self.camera_horizontal_min_length = max(
            0.05,
            float(self._value('camera_horizontal_min_length_ratio')),
        )
        self.camera_vertical_min_length = max(
            0.03,
            float(self._value('camera_vertical_min_length_ratio')),
        )
        self.camera_feature_ema_alpha = float(np.clip(
            float(self._value('camera_feature_ema_alpha')), 0.05, 1.0
        ))
        self.slot2_horizontal_y = float(
            self._value('slot2_horizontal_y_ratio')
        )
        self.slot2_horizontal_angle = float(
            self._value('slot2_horizontal_angle_deg')
        )
        self.slot2_vertical_x = float(
            self._value('slot2_vertical_x_ratio')
        )
        self.slot2_vertical_angle = float(
            self._value('slot2_vertical_angle_deg')
        )
        self.slot3_horizontal_y = float(
            self._value('slot3_horizontal_y_ratio')
        )
        self.slot3_horizontal_angle = float(
            self._value('slot3_horizontal_angle_deg')
        )
        self.slot3_corner_x = float(
            self._value('slot3_corner_x_ratio')
        )
        self.slot3_vertical_angle = float(
            self._value('slot3_vertical_angle_deg')
        )
        self.camera_position_gain = float(
            self._value('camera_position_gain_deg')
        )
        self.camera_vertical_reference_tolerance = max(
            1.0,
            float(self._value(
                'camera_vertical_reference_tolerance_deg'
            )),
        )
        self.camera_horizontal_reference_tolerance = max(
            1.0,
            float(self._value(
                'camera_horizontal_reference_tolerance_deg'
            )),
        )
        self.camera_steer_sign = float(self._value('camera_steer_sign'))
        self.camera_max_steer = int(np.clip(
            int(self._value('camera_max_steer_deg')), 1, 45
        ))
        self.camera_steer_deadband = max(
            0.0, float(self._value('camera_steer_deadband_deg'))
        )
        self.camera_debug_window_name = str(
            self._value('camera_debug_window_name')
        )

        self.rear_hard_stop_angle = math.radians(
            float(self._value('rear_hard_stop_angle_deg'))
        )
        self.rear_hard_stop_distance = float(
            self._value('rear_hard_stop_distance_m')
        )
        self.vehicle_width = float(self._value('vehicle_width_m'))
        self.minimum_side_clearance = float(
            self._value('minimum_side_clearance_m')
        )
        self.lidar_to_rear_bumper = float(
            self._value('lidar_to_rear_bumper_m')
        )

        now = time.monotonic()
        self.startup_delay_deadline = now + self.startup_delay
        self.mode = self.start_mode
        self.state = ParkingState.WAIT_FOR_SCAN
        self.state_started_at = now
        self.last_scan_at: Optional[float] = None
        self.lidar_side_left_m: Optional[float] = None
        self.lidar_side_right_m: Optional[float] = None
        self.lidar_raw_left_m: Optional[float] = None
        self.lidar_raw_right_m: Optional[float] = None
        self.lidar_side_far_frames = 0
        self.lidar_side_gate_frames = 0
        self.lidar_side_gate_seen = False
        self.last_pair_at: Optional[float] = None
        self.last_command = (0, 0)
        self.failure_reason = ''
        self.shutdown_started = False
        self.reverse_segment_started_at: Optional[float] = None
        self.reverse_phase_started_at: Optional[float] = None
        self.reverse_phase = 'IDLE'
        self.reverse_segment_steer = 0
        self.reverse_segment_index = 0
        self.final_correction_count = 0
        self.reverse_segment_drive_duration = self.reverse_segment_duration
        self.lower_vehicle_track_center: Optional[np.ndarray] = None
        self.upper_vehicle_track_center: Optional[np.ndarray] = None
        self.rear_half_lidar_point_count = 0
        self.rear_half_empty_frames = 0
        self.rear_half_points_seen = False
        self.reference_lower_below_count = 0
        self.reference_upper_below_count = 0
        self.reference_lower_missing_frames = 0
        self.reference_upper_missing_frames = 0
        self.reference_lower_gone = False
        self.reference_upper_gone = False
        self.final_completion_tracking_started = False
        self.current_reference_lower: Optional[VehicleCluster] = None
        self.current_reference_upper: Optional[VehicleCluster] = None
        self.final_target_half_gap = self.gap_min_width / 2.0
        self.straight_reverse_latched = False
        self.straight_reverse_started = False
        self.straight_reverse_trigger_distance = math.inf
        self.invalid_scan_count = 0
        self.first_car_frames = 0
        self.gap_frames = 0
        self.latest_scan: Optional[LaserScan] = None
        self.observation = self._empty_observation()
        self.cv_bridge = CvBridge()
        self.camera_line_observation: Optional[ParkingLineObservation] = None
        self.camera_line_confirm_count = 0
        self.camera_line_miss_count = 0
        self.camera_filtered_position_error: Optional[float] = None
        self.camera_debug_image: Optional[np.ndarray] = None
        self.precision_reverse_guidance_source = 'MONITOR_ONLY'

        self.motor_publisher = self.create_publisher(
            Int16MultiArray, self.motor_topic, 10
        )
        self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10
        )
        self.create_timer(1.0 / self.control_hz, self.control_tick)
        if self.debug_view:
            self.create_timer(1.0 / self.debug_hz, self.draw_debug)

        self.get_logger().info(
            f'parking_node_yym: startup stop {self.startup_delay:.1f}s '
            '-> straight approach -> first-car trigger -> '
            f'first-car range<={self.first_car_max_distance:.2f}m and '
            f'point/center-x>={self.recognition_vehicle_min_x:.2f}/'
            f'{self.first_car_min_center_x:.2f}m, '
            f'points>={self.first_car_min_points}, '
            f'extent>={self.first_car_min_extent:.2f}m for '
            f'{self.first_car_confirm_frames} frames -> '
            f'{self.left_turn_duration_sec:.2f}s max-left timed turn '
            f'-> pre-final vehicle x<={self.pre_final_vehicle_max_x:.2f}m '
            'rear-half filter '
            '-> strict gap pair or two-cluster fallback midpoint '
            '-> repeated 1s midpoint-angle reverse corrections '
            f'-> {self.straight_reverse_radius:.2f}m ring stop for '
            f'{self.straight_reverse_stop:.1f}s '
            f'-> {self.final_correction_segment_count} x '
            f'{self.final_correction_duration:.1f}s LiDAR corrections '
            '-> mandatory stop + steer=0 settle '
            '-> steer=0 continuous reverse '
            '-> either original vehicle clears line '
            f'-> PARKED stop {self.exit_wait_after_park:.1f}s '
            '-> timed exit sequence -> EXIT_COMPLETE; '
            f'start_mode={self.start_mode.value}, '
            f'recognition_only={self.recognition_only}'
        )

    def _value(self, name: str):
        return self.get_parameter(name).value

    @staticmethod
    def _empty_observation() -> LidarObservation:
        return LidarObservation(
            False, np.empty((0, 2)), [], [], None, None
        )

    def scan_callback(self, msg: LaserScan) -> None:
        now = time.monotonic()
        self.latest_scan = msg
        self.last_scan_at = now
        observation = self.observe(msg)
        if not observation.scan_valid:
            self.invalid_scan_count += 1
            return

        self.invalid_scan_count = 0
        if (
            self.state == ParkingState.SETTLE_AND_ACQUIRE_GAP
            and observation.pair is None
        ):
            fallback_pair = self.fallback_visible_pair(
                observation.vehicles
            )
            if fallback_pair is not None:
                observation.pair = fallback_pair
                observation.pair_is_fallback = True
        if self.state == ParkingState.REVERSE_CENTER:
            tracked_pair = self.track_parking_pair(
                observation.vehicles,
                max_jump=(
                    self.final_vehicle_track_max_jump
                    if self.straight_reverse_latched
                    else self.vehicle_pair_track_max_jump
                ),
            )
            if tracked_pair is None and not self.straight_reverse_latched:
                tracked_pair = self.fallback_visible_pair(
                    observation.vehicles
                )
            if tracked_pair is not None:
                observation.pair = tracked_pair
        self.observation = observation
        if observation.pair is not None:
            self.last_pair_at = now
        if self.state == ParkingState.REVERSE_CENTER:
            self.update_rear_half_stop_observation(observation)
            self.update_straight_reverse_condition(observation.vehicles)
            if self.final_completion_tracking_started:
                self.update_reference_vehicle_completion(
                    observation.vehicles,
                    observation.pair,
                )
        if self.state == ParkingState.APPROACH_FIRST_CAR:
            close_right_vehicles = [
                vehicle
                for vehicle in observation.right_vehicles
                if (
                    float(vehicle.center[0])
                    >= self.first_car_min_center_x
                    and len(vehicle.points) >= self.first_car_min_points
                    and max(
                        vehicle.x_max - vehicle.x_min,
                        vehicle.y_max - vehicle.y_min,
                    ) >= self.first_car_min_extent
                    and self.vehicle_nearest_distance(vehicle)
                    <= self.first_car_max_distance
                )
            ]
            self.first_car_frames = (
                self.first_car_frames + 1
                if close_right_vehicles else 0
            )
        else:
            self.first_car_frames = 0

        if self.state == ParkingState.SETTLE_AND_ACQUIRE_GAP:
            self.gap_frames = (
                self.gap_frames + 1 if observation.pair is not None else 0
            )
        else:
            self.gap_frames = 0

    def high_camera_callback(self, msg: Image) -> None:
        """Extract the slot-specific white-line pose from the high camera."""
        now = time.monotonic()
        try:
            image = self.cv_bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
            observation, debug_image = self.detect_parking_lines(image, now)
            self.camera_debug_image = debug_image
            if observation is None:
                self.camera_line_miss_count += 1
                if (
                    self.camera_line_miss_count
                    >= self.camera_line_loss_confirm_frames
                ):
                    self.camera_line_confirm_count = 0
                    self.camera_line_observation = None
                    self.camera_filtered_position_error = None
                return
            self.camera_line_miss_count = 0
            self.camera_line_confirm_count = min(
                self.camera_line_confirm_frames,
                self.camera_line_confirm_count + 1,
            )
            self.camera_line_observation = observation
        except Exception as error:
            self.camera_line_miss_count += 1
            self.camera_line_confirm_count = 0
            self.camera_line_observation = None
            self.get_logger().warning(
                f'High camera line processing failed: {error}',
                throttle_duration_sec=2.0,
            )

    @staticmethod
    def normalized_line_angle(
        line: tuple[float, float, float, float]
    ) -> float:
        """Return a direction-independent image-line angle in [-90, 90)."""
        x1, y1, x2, y2 = line
        angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
        while angle >= 90.0:
            angle -= 180.0
        while angle < -90.0:
            angle += 180.0
        return angle

    @staticmethod
    def vertical_line_angle(
        line: tuple[float, float, float, float]
    ) -> float:
        """Return image tilt from vertical, consistently measured top-bottom."""
        x1, y1, x2, y2 = line
        if y1 > y2:
            x1, y1, x2, y2 = x2, y2, x1, y1
        return math.degrees(math.atan2(x2 - x1, max(1.0, y2 - y1)))

    @staticmethod
    def line_x_at_y(
        line: tuple[float, float, float, float], y: float
    ) -> Optional[float]:
        x1, y1, x2, y2 = line
        dy = y2 - y1
        if abs(dy) < 1.0e-6:
            return None
        return x1 + (y - y1) * (x2 - x1) / dy

    @staticmethod
    def line_y_at_x(
        line: tuple[float, float, float, float], x: float
    ) -> Optional[float]:
        x1, y1, x2, y2 = line
        dx = x2 - x1
        if abs(dx) < 1.0e-6:
            return None
        return y1 + (x - x1) * (y2 - y1) / dx

    @staticmethod
    def line_intersection(
        first: tuple[float, float, float, float],
        second: tuple[float, float, float, float],
    ) -> Optional[tuple[float, float]]:
        x1, y1, x2, y2 = first
        x3, y3, x4, y4 = second
        denominator = (
            (x1 - x2) * (y3 - y4)
            - (y1 - y2) * (x3 - x4)
        )
        if abs(denominator) < 1.0e-6:
            return None
        first_cross = x1 * y2 - y1 * x2
        second_cross = x3 * y4 - y3 * x4
        x = (
            first_cross * (x3 - x4)
            - (x1 - x2) * second_cross
        ) / denominator
        y = (
            first_cross * (y3 - y4)
            - (y1 - y2) * second_cross
        ) / denominator
        return float(x), float(y)

    def detect_parking_lines(
        self, image: np.ndarray, now: float
    ) -> tuple[Optional[ParkingLineObservation], np.ndarray]:
        """Detect the horizontal boundary and slot-specific longitudinal line."""
        debug = image.copy()
        if image.size == 0:
            return None, debug
        height, width = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(
            hsv,
            np.array([0, 0, self.camera_line_white_threshold], dtype=np.uint8),
            np.array(
                [180, self.camera_line_max_saturation, 255], dtype=np.uint8
            ),
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        white_mask = cv2.morphologyEx(
            white_mask, cv2.MORPH_CLOSE, kernel, iterations=2
        )
        edges = cv2.Canny(white_mask, 40, 120)
        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180.0,
            25,
            minLineLength=max(12, int(height * 0.06)),
            maxLineGap=max(10, int(height * 0.08)),
        )
        if lines is None:
            self._draw_camera_reference(debug)
            return None, debug

        horizontal_target_y = (
            self.slot2_horizontal_y
            if self.parking_slot_number == 2
            else self.slot3_horizontal_y
        )
        horizontal_candidates: list[
            tuple[float, tuple[float, float, float, float]]
        ] = []
        vertical_candidates: list[
            tuple[float, tuple[float, float, float, float]]
        ] = []
        for raw_line in lines[:, 0]:
            x1, y1, x2, y2 = (float(value) for value in raw_line)
            line = (x1, y1, x2, y2)
            length = math.hypot(x2 - x1, y2 - y1)
            horizontal_angle = self.normalized_line_angle(line)
            if (
                abs(horizontal_angle) <= self.camera_horizontal_max_angle
                and length >= width * self.camera_horizontal_min_length
                and min(y1, y2) <= height * 0.70
            ):
                middle_y = 0.5 * (y1 + y2)
                target_distance = abs(
                    middle_y / max(1, height) - horizontal_target_y
                )
                score = length / (1.0 + 1.5 * target_distance)
                horizontal_candidates.append((score, line))

            vertical_angle = self.vertical_line_angle(line)
            if (
                abs(vertical_angle) <= self.camera_vertical_max_angle
                and length >= height * self.camera_vertical_min_length
            ):
                middle_x = 0.5 * (x1 + x2) / max(1, width)
                if self.parking_slot_number == 2:
                    if not 0.20 <= middle_x <= 0.80:
                        continue
                    position_distance = abs(
                        middle_x - self.slot2_vertical_x
                    )
                    angle_distance = abs(
                        vertical_angle - self.slot2_vertical_angle
                    ) / 90.0
                else:
                    if not 0.45 <= middle_x <= 0.98:
                        continue
                    position_distance = abs(
                        middle_x - self.slot3_corner_x
                    )
                    angle_distance = abs(
                        vertical_angle - self.slot3_vertical_angle
                    ) / 90.0
                score = length / (
                    1.0 + 2.0 * position_distance + angle_distance
                )
                vertical_candidates.append((score, line))

        if not horizontal_candidates or not vertical_candidates:
            self._draw_camera_reference(debug)
            return None, debug

        # Pick a connected-looking T/corner pair instead of selecting the two
        # line types independently. The requested parking boundary is always
        # below the visible longitudinal line: in image coordinates its
        # intersection must be at, or just below, the vertical segment's
        # lower endpoint. This rejects the other white horizontal course line.
        best_pair: Optional[tuple[
            float,
            tuple[float, float, float, float],
            tuple[float, float, float, float],
            tuple[float, float],
        ]] = None
        lower_endpoint_tolerance = max(8.0, height * 0.04)
        maximum_endpoint_gap = max(18.0, height * 0.15)
        for horizontal_score, candidate_horizontal in horizontal_candidates:
            for vertical_score, candidate_vertical in vertical_candidates:
                candidate_intersection = self.line_intersection(
                    candidate_horizontal, candidate_vertical
                )
                if candidate_intersection is None:
                    continue
                candidate_x, candidate_y = candidate_intersection
                if not (
                    -0.10 * width <= candidate_x <= 1.10 * width
                    and -0.10 * height <= candidate_y <= 0.80 * height
                ):
                    continue
                vertical_y1 = candidate_vertical[1]
                vertical_y2 = candidate_vertical[3]
                vertical_middle_y = 0.5 * (vertical_y1 + vertical_y2)
                vertical_bottom_y = max(vertical_y1, vertical_y2)
                if candidate_y < vertical_middle_y:
                    continue
                if candidate_y < vertical_bottom_y - lower_endpoint_tolerance:
                    continue
                endpoint_gap = candidate_y - vertical_bottom_y
                if endpoint_gap > maximum_endpoint_gap:
                    continue
                target_distance = abs(
                    candidate_y / max(1, height) - horizontal_target_y
                )
                pair_score = (
                    horizontal_score + vertical_score
                ) / (
                    1.0
                    + 3.0 * target_distance
                    + 4.0 * abs(endpoint_gap) / max(1, height)
                )
                if best_pair is None or pair_score > best_pair[0]:
                    best_pair = (
                        pair_score,
                        candidate_horizontal,
                        candidate_vertical,
                        candidate_intersection,
                    )

        if best_pair is None:
            self._draw_camera_reference(debug)
            return None, debug
        _, horizontal_line, vertical_line, intersection = best_pair
        corner_x, corner_y = intersection

        horizontal_angle = self.normalized_line_angle(horizontal_line)
        vertical_angle = self.vertical_line_angle(vertical_line)
        if self.parking_slot_number == 2:
            position_error = corner_x / width - self.slot2_vertical_x
            target_vertical_angle = self.slot2_vertical_angle
            target_horizontal_angle = self.slot2_horizontal_angle
            horizontal_y = corner_y
        else:
            position_error = corner_x / width - self.slot3_corner_x
            target_vertical_angle = self.slot3_vertical_angle
            target_horizontal_angle = self.slot3_horizontal_angle
            horizontal_y = corner_y
        if horizontal_y is None:
            self._draw_camera_reference(debug)
            return None, debug

        vertical_angle_error = vertical_angle - target_vertical_angle
        horizontal_angle_error = horizontal_angle - target_horizontal_angle
        # The line angles validate that this is the intended parking corner;
        # they no longer contribute to steering. One intersection point cannot
        # independently control yaw, so angle errors are used only to reject a
        # false crossing before the x-coordinate visual servo is activated.
        if (
            abs(vertical_angle_error)
            > self.camera_vertical_reference_tolerance
            or abs(horizontal_angle_error)
            > self.camera_horizontal_reference_tolerance
        ):
            self._draw_camera_reference(debug)
            return None, debug
        alpha = self.camera_feature_ema_alpha
        if self.camera_filtered_position_error is None:
            filtered_position = position_error
        else:
            filtered_position = (
                alpha * position_error
                + (1.0 - alpha) * self.camera_filtered_position_error
            )
        self.camera_filtered_position_error = filtered_position

        raw_steering = (
            self.camera_steer_sign
            * self.camera_position_gain
            * filtered_position
        )
        if abs(raw_steering) < self.camera_steer_deadband:
            raw_steering = 0.0
        steering = int(round(np.clip(
            raw_steering,
            -self.camera_max_steer,
            self.camera_max_steer,
        )))
        observation = ParkingLineObservation(
            stamp=now,
            horizontal_line=horizontal_line,
            vertical_line=vertical_line,
            intersection=(corner_x, corner_y),
            horizontal_y_ratio=float(horizontal_y / height),
            horizontal_angle_deg=horizontal_angle,
            vertical_angle_deg=vertical_angle,
            position_error_ratio=float(filtered_position),
            steering_deg=steering,
        )
        self._draw_camera_reference(debug)
        self._draw_detected_camera_lines(debug, observation)
        return observation, debug

    def _draw_camera_reference(self, image: np.ndarray) -> None:
        height, width = image.shape[:2]
        if self.parking_slot_number == 2:
            target_x = int(round(self.slot2_vertical_x * width))
            target_y = int(round(self.slot2_horizontal_y * height))
        else:
            target_x = int(round(self.slot3_corner_x * width))
            target_y = int(round(self.slot3_horizontal_y * height))
        cv2.circle(image, (target_x, target_y), 13, (0, 0, 255), 3)
        cv2.drawMarker(
            image,
            (target_x, target_y),
            (0, 0, 255),
            cv2.MARKER_CROSS,
            28,
            3,
        )
        cv2.putText(
            image,
            'TARGET',
            (target_x + 16, target_y + 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

    def _draw_detected_camera_lines(
        self, image: np.ndarray, observation: ParkingLineObservation
    ) -> None:
        height, width = image.shape[:2]
        target = (
            int(round(
                (
                    self.slot2_vertical_x
                    if self.parking_slot_number == 2
                    else self.slot3_corner_x
                ) * width
            )),
            int(round(
                (
                    self.slot2_horizontal_y
                    if self.parking_slot_number == 2
                    else self.slot3_horizontal_y
                ) * height
            )),
        )
        detected = (
            int(round(observation.intersection[0])),
            int(round(observation.intersection[1])),
        )
        cv2.line(image, target, detected, (0, 255, 255), 2)
        cv2.circle(image, detected, 11, (0, 255, 0), -1)
        cv2.circle(image, detected, 14, (0, 0, 0), 2)
        cv2.putText(
            image,
            'NOW',
            (detected[0] + 15, detected[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f'slot={self.parking_slot_number} point steer='
            f'{observation.steering_deg:+d} xerr='
            f'{observation.position_error_ratio:+.3f} '
            f'xy=({observation.intersection[0] / width:.3f},'
            f'{observation.intersection[1] / height:.3f})',
            (10, image.shape[0] - 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

    def active_camera_line_observation(
        self, now: Optional[float] = None
    ) -> Optional[ParkingLineObservation]:
        """Return only a confirmed and fresh paired-line observation."""
        if now is None:
            now = time.monotonic()
        observation = self.camera_line_observation
        if (
            observation is None
            or self.camera_line_confirm_count < self.camera_line_confirm_frames
            or now - observation.stamp > self.camera_line_timeout
        ):
            return None
        return observation

    def lidar_side_vehicle_distances(
        self, vehicles: list[VehicleCluster]
    ) -> Optional[tuple[float, float]]:
        """Return -90/+90 distances from two distinct vehicle clusters."""
        left_candidates: list[tuple[int, float]] = []
        right_candidates: list[tuple[int, float]] = []
        half_width = self.lidar_side_gate_half_width

        for index, vehicle in enumerate(vehicles):
            if len(vehicle.points) == 0:
                continue
            scan_angles = np.arctan2(
                -vehicle.points[:, 1],
                -vehicle.points[:, 0],
            )
            left_count = int(np.count_nonzero(
                np.abs(scan_angles + math.pi / 2.0) <= half_width
            ))
            right_count = int(np.count_nonzero(
                np.abs(scan_angles - math.pi / 2.0) <= half_width
            ))
            if left_count >= self.lidar_side_gate_min_points:
                left_mask = (
                    np.abs(scan_angles + math.pi / 2.0) <= half_width
                )
                left_candidates.append((
                    index,
                    float(np.median(np.linalg.norm(
                        vehicle.points[left_mask], axis=1
                    ))),
                ))
            if right_count >= self.lidar_side_gate_min_points:
                right_mask = (
                    np.abs(scan_angles - math.pi / 2.0) <= half_width
                )
                right_candidates.append((
                    index,
                    float(np.median(np.linalg.norm(
                        vehicle.points[right_mask], axis=1
                    ))),
                ))

        distinct_pairs = [
            (left_distance + right_distance, left_distance, right_distance)
            for left_index, left_distance in left_candidates
            for right_index, right_distance in right_candidates
            if left_index != right_index
        ]
        if not distinct_pairs:
            return None
        _, left_distance, right_distance = min(distinct_pairs)
        return left_distance, right_distance

    def lidar_raw_side_distances(
        self, msg: LaserScan
    ) -> tuple[Optional[float], Optional[float]]:
        """Return raw median ranges near scan -90 and +90 degrees."""
        left_ranges: list[float] = []
        right_ranges: list[float] = []
        for index, raw_distance in enumerate(msg.ranges):
            distance = float(raw_distance)
            if (
                not math.isfinite(distance)
                or distance < msg.range_min
                or distance > msg.range_max
            ):
                continue
            angle = msg.angle_min + index * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if (
                abs(angle + math.pi / 2.0)
                <= self.lidar_side_far_half_width
            ):
                left_ranges.append(distance)
            if (
                abs(angle - math.pi / 2.0)
                <= self.lidar_side_far_half_width
            ):
                right_ranges.append(distance)
        return (
            float(np.median(left_ranges)) if left_ranges else None,
            float(np.median(right_ranges)) if right_ranges else None,
        )

    def held_steering_command(self) -> int:
        """Reuse the last steering target already sent through /motor_control."""
        return int(self.last_command[0])

    def update_rear_half_stop_observation(
        self, observation: LidarObservation
    ) -> None:
        """Track all valid LiDAR points below the green horizontal x=0 line."""
        self.rear_half_lidar_point_count = int(np.count_nonzero(
            observation.points[:, 0] < -self.rear_half_stop_margin
        ))
        if self.rear_half_lidar_point_count > 0:
            self.rear_half_points_seen = True
            self.rear_half_empty_frames = 0
        elif (
            self.rear_half_points_seen
            or self.straight_reverse_started
        ):
            self.rear_half_empty_frames += 1

    def match_reference_vehicle(
        self,
        vehicles: list[VehicleCluster],
        target_center: Optional[np.ndarray],
        excluded: Optional[VehicleCluster] = None,
    ) -> Optional[VehicleCluster]:
        """Match one original parked vehicle without adopting a distant unit."""
        if target_center is None:
            return None
        candidates = [
            (
                float(np.linalg.norm(vehicle.center - target_center)),
                vehicle,
            )
            for vehicle in vehicles
            if vehicle is not excluded
        ]
        if not candidates:
            return None
        distance, vehicle = min(candidates, key=lambda item: item[0])
        if distance > self.final_vehicle_track_max_jump:
            return None
        return vehicle

    def update_reference_vehicle_completion(
        self,
        vehicles: list[VehicleCluster],
        pair: Optional[ParkingPair],
    ) -> None:
        """Latch each original vehicle gone once its below-line points vanish."""
        if pair is not None:
            lower_vehicle = pair.lower
            upper_vehicle = pair.upper
        else:
            lower_vehicle = self.match_reference_vehicle(
                vehicles, self.lower_vehicle_track_center
            )
            upper_vehicle = self.match_reference_vehicle(
                vehicles,
                self.upper_vehicle_track_center,
                excluded=lower_vehicle,
            )

        self.current_reference_lower = (
            None if self.reference_lower_gone else lower_vehicle
        )
        self.current_reference_upper = (
            None if self.reference_upper_gone else upper_vehicle
        )
        if lower_vehicle is not None:
            self.lower_vehicle_track_center = lower_vehicle.center.copy()
        if upper_vehicle is not None:
            self.upper_vehicle_track_center = upper_vehicle.center.copy()

        self.reference_lower_below_count = (
            0
            if lower_vehicle is None
            else int(np.count_nonzero(
                lower_vehicle.points[:, 0] < -self.rear_half_stop_margin
            ))
        )
        self.reference_upper_below_count = (
            0
            if upper_vehicle is None
            else int(np.count_nonzero(
                upper_vehicle.points[:, 0] < -self.rear_half_stop_margin
            ))
        )

        if not self.reference_lower_gone:
            if self.reference_lower_below_count > 0:
                self.reference_lower_missing_frames = 0
            else:
                self.reference_lower_missing_frames += 1
                if (
                    self.reference_lower_missing_frames
                    >= self.rear_half_empty_confirm_frames
                ):
                    self.reference_lower_gone = True
                    self.get_logger().info(
                        'Reference lower/right vehicle cleared the green '
                        'horizontal line; later objects on that side ignored'
                    )

        if not self.reference_upper_gone:
            if self.reference_upper_below_count > 0:
                self.reference_upper_missing_frames = 0
            else:
                self.reference_upper_missing_frames += 1
                if (
                    self.reference_upper_missing_frames
                    >= self.rear_half_empty_confirm_frames
                ):
                    self.reference_upper_gone = True
                    self.get_logger().info(
                        'Reference upper/left vehicle cleared the green '
                        'horizontal line; later objects on that side ignored'
                    )
        if self.reference_lower_gone:
            self.current_reference_lower = None
        if self.reference_upper_gone:
            self.current_reference_upper = None

    def update_straight_reverse_condition(
        self, vehicles: list[VehicleCluster]
    ) -> None:
        """Latch when any obstacle-vehicle point enters the second ring."""
        if (
            self.straight_reverse_latched
            or self.reverse_phase not in (
                'STEER_SETTLE',
                'DRIVE',
                'MEASURE_STOP',
            )
        ):
            return
        if not vehicles:
            self.straight_reverse_trigger_distance = math.inf
            return

        self.straight_reverse_trigger_distance = min(
            self.vehicle_nearest_distance(vehicle)
            for vehicle in vehicles
        )
        if (
            self.straight_reverse_trigger_distance
            <= self.straight_reverse_radius
        ):
            self.straight_reverse_latched = True
            self.straight_reverse_started = False
            self.final_correction_count = 0
            if self.observation.pair is not None:
                self.final_target_half_gap = max(
                    self.vehicle_width / 2.0,
                    self.observation.pair.gap_width / 2.0,
                )
            self.reverse_phase = 'FINAL_STOP'
            self.reverse_phase_started_at = time.monotonic()
            self.get_logger().info(
                'Final correction latched: a parked-vehicle point entered '
                f'the {self.straight_reverse_radius:.2f}m ring; stopping for '
                f'{self.straight_reverse_stop:.1f}s with steer=0 before '
                'the final two LiDAR corrections.'
            )

    def observe(self, msg: LaserScan) -> LidarObservation:
        points: list[tuple[float, float]] = []
        rear_distances: list[float] = []
        scan_point_count = 0
        vehicle_cluster_max_range = (
            self.initial_gap_cluster_max_range
            if self.state == ParkingState.SETTLE_AND_ACQUIRE_GAP
            else self.cluster_max_range
        )

        for index, raw_distance in enumerate(msg.ranges):
            distance = float(raw_distance)
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            scan_point_count += 1
            if distance < self.parking_min_range:
                continue

            angle = msg.angle_min + index * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if (
                abs(angle) <= self.rear_hard_stop_angle
                and distance <= self.cluster_max_range
            ):
                rear_distances.append(distance)
            if (
                abs(angle) > self.valid_sector_max_abs
                or distance > vehicle_cluster_max_range
            ):
                continue

            # Vehicle coordinates: +x forward, +y left.
            points.append(
                (-distance * math.cos(angle), -distance * math.sin(angle))
            )

        point_array = (
            np.asarray(points, dtype=np.float64)
            if points else np.empty((0, 2), dtype=np.float64)
        )
        vehicle_point_array = point_array
        if self.mode == ParkingMode.RECOGNITION:
            vehicle_point_array = point_array[
                point_array[:, 0] >= self.recognition_vehicle_min_x
            ]
        elif (
            self.mode == ParkingMode.PARKING
            and not self.straight_reverse_latched
        ):
            vehicle_point_array = point_array[
                point_array[:, 0] <= self.pre_final_vehicle_max_x
            ]
        vehicles = [
            self.make_vehicle_cluster(component)
            for component in self.connected_components(vehicle_point_array)
        ]
        vehicles = [
            vehicle for vehicle in vehicles
            if max(
                vehicle.x_max - vehicle.x_min,
                vehicle.y_max - vehicle.y_min,
            ) >= self.obstacle_min_extent
        ]
        vehicles.sort(key=lambda item: len(item.points), reverse=True)
        right_vehicles = [
            item for item in vehicles
            if item.center[1] < -self.right_detection_margin
        ]
        pair = self.select_parking_pair(vehicles)
        observation_points = (
            vehicle_point_array
            if self.mode == ParkingMode.RECOGNITION
            else point_array
        )
        return LidarObservation(
            scan_valid=scan_point_count >= self.scan_quality_min_points,
            points=observation_points,
            vehicles=vehicles,
            right_vehicles=right_vehicles,
            pair=pair,
            rear_min_distance=(
                min(rear_distances) if rear_distances else None
            ),
        )

    def connected_components(self, points: np.ndarray) -> list[np.ndarray]:
        """Region-grow nearby LiDAR returns into physical obstacle bundles."""
        if len(points) == 0:
            return []
        unassigned = set(range(len(points)))
        components: list[np.ndarray] = []
        while unassigned:
            seed = unassigned.pop()
            component = [seed]
            pending = [seed]
            while pending:
                current = pending.pop()
                candidates = np.fromiter(unassigned, dtype=np.intp)
                if len(candidates) == 0:
                    continue
                distances = np.linalg.norm(
                    points[candidates] - points[current], axis=1
                )
                nearby = candidates[
                    distances <= self.cluster_neighbor_distance
                ]
                for neighbor in nearby:
                    neighbor_index = int(neighbor)
                    unassigned.remove(neighbor_index)
                    component.append(neighbor_index)
                    pending.append(neighbor_index)
            if len(component) >= self.cluster_min_points:
                components.append(points[component])
        return components

    @staticmethod
    def make_vehicle_cluster(points: np.ndarray) -> VehicleCluster:
        center = np.median(points, axis=0)
        centered = points - center
        covariance = centered.T @ centered
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        axis = eigenvectors[:, int(np.argmax(eigenvalues))]
        if axis[0] < 0.0:
            axis = -axis
        axis_angle = math.atan2(float(axis[1]), float(axis[0]))
        return VehicleCluster(
            points=points,
            center=center,
            axis_angle=axis_angle,
            x_min=float(np.quantile(points[:, 0], 0.05)),
            x_max=float(np.quantile(points[:, 0], 0.95)),
            y_min=float(np.quantile(points[:, 1], 0.05)),
            y_max=float(np.quantile(points[:, 1], 0.95)),
        )

    def select_parking_pair(
        self, vehicles: list[VehicleCluster]
    ) -> Optional[ParkingPair]:
        """Choose the adjacent vehicles whose free gap is nearest the ego."""
        if len(vehicles) < 2:
            return None

        ordered = sorted(vehicles, key=lambda item: item.center[1])
        candidates: list[tuple[float, ParkingPair]] = []
        for lower, upper in zip(ordered, ordered[1:]):
            pair = self.build_parking_pair(
                lower, upper, require_valid_gap=True
            )
            if pair is None:
                continue
            score = abs(pair.gap_center_y) + 0.15 * (
                np.linalg.norm(lower.center)
                + np.linalg.norm(upper.center)
            )
            candidates.append((float(score), pair))
        if not candidates:
            return None
        return min(candidates, key=lambda item: item[0])[1]

    def build_parking_pair(
        self,
        first: VehicleCluster,
        second: VehicleCluster,
        *,
        require_valid_gap: bool,
    ) -> Optional[ParkingPair]:
        lower, upper = sorted(
            (first, second), key=lambda item: item.center[1]
        )
        gap_low = lower.y_max
        gap_high = upper.y_min
        gap_width = gap_high - gap_low
        gap_center = 0.5 * (gap_low + gap_high)
        if require_valid_gap and not (
            self.gap_min_width <= gap_width <= self.gap_max_width
            and abs(gap_center) <= self.gap_track_max_center
        ):
            return None

        return ParkingPair(
            lower=lower,
            upper=upper,
            # The red reference point in the debug view: midpoint of the two
            # obstacle representative points (median cluster centers).
            reference_point=0.5 * (lower.center + upper.center),
            gap_center_y=gap_center,
            gap_width=gap_width,
            left_clearance=gap_high - self.vehicle_width / 2.0,
            right_clearance=-self.vehicle_width / 2.0 - gap_low,
        )

    def track_parking_pair(
        self,
        vehicles: list[VehicleCluster],
        *,
        max_jump: Optional[float] = None,
    ) -> Optional[ParkingPair]:
        """Keep both original vehicles even when the strict gap test fails."""
        if (
            len(vehicles) < 2
            or self.lower_vehicle_track_center is None
            or self.upper_vehicle_track_center is None
        ):
            return None

        allowed_jump = (
            self.vehicle_pair_track_max_jump
            if max_jump is None
            else max_jump
        )
        best = None
        for lower_index, lower in enumerate(vehicles):
            for upper_index, upper in enumerate(vehicles):
                if lower_index == upper_index:
                    continue
                lower_jump = float(np.linalg.norm(
                    lower.center - self.lower_vehicle_track_center
                ))
                upper_jump = float(np.linalg.norm(
                    upper.center - self.upper_vehicle_track_center
                ))
                if (
                    lower_jump > allowed_jump
                    or upper_jump > allowed_jump
                ):
                    continue
                cost = lower_jump + upper_jump
                if best is None or cost < best[0]:
                    best = (cost, lower, upper)
        if best is None:
            return None

        pair = self.build_parking_pair(
            best[1], best[2], require_valid_gap=False
        )
        if self.final_completion_tracking_started:
            pair_text = (
                f'original L/R below='
                f'{self.reference_upper_below_count}/'
                f'{self.reference_lower_below_count} '
                f'missing={self.reference_upper_missing_frames}/'
                f'{self.reference_lower_missing_frames} '
                f'gone={int(self.reference_upper_gone)}/'
                f'{int(self.reference_lower_gone)}'
            )
        elif pair is None:
            return None
        self.lower_vehicle_track_center = pair.lower.center.copy()
        self.upper_vehicle_track_center = pair.upper.center.copy()
        return pair

    def fallback_visible_pair(
        self, vehicles: list[VehicleCluster]
    ) -> Optional[ParkingPair]:
        """Use the two visible clusters around the rear centerline.

        This keeps steering updates alive when perspective makes the original
        strict gap-width or inter-frame tracking test fail even though both
        parked vehicles are still plainly visible.
        """
        if len(vehicles) < 2:
            return None

        candidates: list[tuple[float, ParkingPair]] = []
        for first_index, first in enumerate(vehicles):
            for second in vehicles[first_index + 1:]:
                pair = self.build_parking_pair(
                    first, second, require_valid_gap=False
                )
                if pair is None:
                    continue
                straddles_center = (
                    pair.lower.center[1] <= 0.0
                    <= pair.upper.center[1]
                )
                score = (
                    (0.0 if straddles_center else 10.0)
                    + abs(float(pair.reference_point[1]))
                    + 0.05 * (
                        float(np.linalg.norm(pair.lower.center))
                        + float(np.linalg.norm(pair.upper.center))
                    )
                )
                candidates.append((score, pair))
        if not candidates:
            return None

        pair = min(candidates, key=lambda item: item[0])[1]
        self.lower_vehicle_track_center = pair.lower.center.copy()
        self.upper_vehicle_track_center = pair.upper.center.copy()
        return pair

    def transition(self, next_state: ParkingState, now: float) -> None:
        if next_state == self.state:
            return
        self.get_logger().info(
            f'YYM parking: {self.state.value} -> {next_state.value}'
        )
        self.state = next_state
        self.state_started_at = now
        if next_state == ParkingState.SETTLE_AND_ACQUIRE_GAP:
            self.gap_frames = 0
        elif next_state == ParkingState.REVERSE_CENTER:
            self.last_pair_at = now
            pair = self.observation.pair
            self.lower_vehicle_track_center = (
                pair.lower.center.copy() if pair is not None else None
            )
            self.upper_vehicle_track_center = (
                pair.upper.center.copy() if pair is not None else None
            )
            self.rear_half_lidar_point_count = int(np.count_nonzero(
                self.observation.points[:, 0]
                < -self.rear_half_stop_margin
            ))
            self.rear_half_points_seen = (
                self.rear_half_lidar_point_count > 0
            )
            self.rear_half_empty_frames = 0
            self.reference_lower_below_count = 0
            self.reference_upper_below_count = 0
            self.reference_lower_missing_frames = 0
            self.reference_upper_missing_frames = 0
            self.reference_lower_gone = False
            self.reference_upper_gone = False
            self.final_completion_tracking_started = False
            self.current_reference_lower = (
                pair.lower if pair is not None else None
            )
            self.current_reference_upper = (
                pair.upper if pair is not None else None
            )
            self.final_target_half_gap = (
                max(self.vehicle_width / 2.0, pair.gap_width / 2.0)
                if pair is not None
                else self.gap_min_width / 2.0
            )
            self.straight_reverse_latched = False
            self.straight_reverse_started = False
            self.final_correction_count = 0
            self.straight_reverse_trigger_distance = math.inf
            self.lidar_side_gate_frames = 0
            self.lidar_side_left_m = None
            self.lidar_side_right_m = None
            self.lidar_raw_left_m = None
            self.lidar_raw_right_m = None
            self.lidar_side_far_frames = 0
            self.lidar_side_gate_seen = False
            self.reverse_phase = 'STEER_SETTLE'
            self.reverse_phase_started_at = now
            self.reverse_segment_started_at = None
            self.reverse_segment_index = 1
            self.reverse_segment_drive_duration = self.reverse_segment_duration
            self.reverse_segment_steer = (
                self.pre_final_reverse_steering(pair)
                if pair is not None
                else 0
            )
            self.get_logger().info(
                f'LiDAR reverse segment 1: steer='
                f'{self.reverse_segment_steer}deg for '
                f'{self.reverse_segment_drive_duration:.1f}s; '
                'waiting for vehicles at both +/-90deg side gates'
            )

    def enter_parking_mode(self, now: float) -> None:
        if self.mode != ParkingMode.PARKING:
            self.get_logger().info(
                'YYM mode: RECOGNITION -> PARKING'
            )
        self.mode = ParkingMode.PARKING
        self.transition(ParkingState.SETTLE_AND_ACQUIRE_GAP, now)

    def control_tick(self) -> None:
        now = time.monotonic()
        if self.state in (
            ParkingState.EXIT_COMPLETE,
            ParkingState.PARKING_FAILED,
            ParkingState.EMERGENCY_STOP,
        ):
            # Terminal states are latched. Avoid repeatedly logging the same
            # LiDAR timeout/failure at the 20 Hz control rate.
            self.reverse_phase = f'{self.state.value}_HOLD'
            self.publish_control(0, 0)
            return
        if self.last_scan_at is None:
            # Do not send a 0-deg target while waiting to start. With no input,
            # drive_control keeps both PWM outputs at zero, so the stationary
            # wheels remain at their current angle.
            return
        if now - self.last_scan_at > self.scan_timeout:
            self.get_logger().error('LiDAR timeout: emergency stop')
            self.transition(ParkingState.EMERGENCY_STOP, now)
            self.publish_control(0, 0)
            return
        if self.invalid_scan_count > 0:
            if self.state == ParkingState.WAIT_FOR_SCAN:
                # Same startup behavior for invalid initial scans: leave the
                # stationary steering untouched until a valid scan arrives.
                return
            if self.invalid_scan_count >= self.invalid_scan_confirm_frames:
                self.get_logger().error('Invalid LiDAR stream: emergency stop')
                self.transition(ParkingState.EMERGENCY_STOP, now)
            self.publish_control(0, 0)
            return

        if self.state == ParkingState.WAIT_FOR_SCAN:
            self.transition(ParkingState.START_DELAY, now)
            self.publish_control(0, 0)
            return

        if self.state == ParkingState.START_DELAY:
            remaining = max(0.0, self.startup_delay_deadline - now)
            self.reverse_phase = f'START_DELAY_{remaining:.1f}s'
            self.publish_control(0, 0)
            if remaining <= 0.0:
                if self.mode == ParkingMode.PARKING:
                    self.transition(
                        ParkingState.SETTLE_AND_ACQUIRE_GAP, now
                    )
                else:
                    self.transition(ParkingState.APPROACH_FIRST_CAR, now)
                self.get_logger().info(
                    'Startup delay complete: beginning driving sequence'
                )
            return

        elapsed = now - self.state_started_at
        if self.state == ParkingState.APPROACH_FIRST_CAR:
            if elapsed >= self.approach_timeout:
                self.fail('first parked vehicle detection timeout', now)
                return
            if self.first_car_frames >= self.first_car_confirm_frames:
                self.transition(ParkingState.SET_LEFT_STEER, now)
                self.publish_control(self.left_max_steer, 0)
                return
            # Approach always requests an explicit straight wheel angle.
            self.publish_control(0, self.approach_speed)
            return

        if self.state == ParkingState.SET_LEFT_STEER:
            if elapsed >= self.steer_settle_sec:
                self.transition(ParkingState.TURN_LEFT_TIMED, now)
                self.publish_control(
                    self.left_max_steer, self.turn_speed
                )
                return
            self.publish_control(self.left_max_steer, 0)
            return

        if self.state == ParkingState.TURN_LEFT_TIMED:
            if elapsed >= self.left_turn_duration_sec:
                if self.recognition_only:
                    self.transition(
                        ParkingState.RECOGNITION_COMPLETE, now
                    )
                    self.get_logger().info(
                        'Recognition complete: timed left turn finished; '
                        'stopping before automatic shutdown'
                    )
                else:
                    self.enter_parking_mode(now)
                self.publish_control(0, 0)
                return
            self.publish_control(self.left_max_steer, self.turn_speed)
            return

        if self.state == ParkingState.RECOGNITION_COMPLETE:
            self.publish_control(0, 0)
            if elapsed >= self.recognition_shutdown_delay_sec:
                self.shutdown_program()
            return

        if self.state == ParkingState.SETTLE_AND_ACQUIRE_GAP:
            if elapsed >= self.gap_acquire_timeout:
                self.fail('two-vehicle parking gap acquisition timeout', now)
                return
            if (
                elapsed >= self.steer_settle_sec
                and self.gap_frames >= self.gap_confirm_frames
            ):
                pair = self.observation.pair
                if pair is None:
                    self.publish_control(0, 0)
                    return
                required_width = (
                    self.vehicle_width + 2.0 * self.minimum_side_clearance
                )
                if (
                    not self.observation.pair_is_fallback
                    and pair.gap_width < required_width
                ):
                    self.fail(
                        f'gap too narrow: {pair.gap_width:.2f} m '
                        f'< {required_width:.2f} m',
                        now,
                    )
                    return
                if self.observation.pair_is_fallback:
                    self.get_logger().warning(
                        'Starting reverse with two-cluster fallback midpoint; '
                        'strict gap pair remains preferred when available'
                    )
                self.transition(ParkingState.REVERSE_CENTER, now)
            self.publish_control(0, 0)
            return

        if self.state == ParkingState.REVERSE_CENTER:
            if (
                self.reverse_timeout > 0.0
                and elapsed >= self.reverse_timeout
            ):
                self.fail('LiDAR reverse/exit timeout', now)
                return

            lidar_reverse_phases = (
                'STEER_SETTLE',
                'DRIVE',
                'MEASURE_STOP',
            )
            if (
                self.reverse_phase in lidar_reverse_phases
                and self.observation.rear_min_distance is not None
                and self.observation.rear_min_distance
                <= self.rear_hard_stop_distance
            ):
                self.fail('rear obstacle inside hard-stop distance', now)
                return

            phase_elapsed = (
                0.0
                if self.reverse_phase_started_at is None
                else now - self.reverse_phase_started_at
            )
            if self.reverse_phase == 'FINAL_STOP':
                self.precision_reverse_guidance_source = 'WAITING_5S'
                self.publish_control(0, 0)
                if phase_elapsed >= self.straight_reverse_stop:
                    self.reverse_phase = 'FINAL_MEASURE_STOP'
                    self.reverse_phase_started_at = now
                    self.straight_reverse_started = True
                    self.reverse_segment_index = 0
                    self.final_correction_count = 0
                    self.final_completion_tracking_started = True
                    self.reference_lower_missing_frames = 0
                    self.reference_upper_missing_frames = 0
                    self.reference_lower_gone = False
                    self.reference_upper_gone = False
                    self.get_logger().info(
                        'Five-second stop complete: starting '
                        f'{self.final_correction_segment_count} x '
                        f'{self.final_correction_duration:.1f}s LiDAR '
                        'corrections using line angle when misaligned or '
                        'average parked-vehicle tilt when aligned'
                    )
                return

            final_reverse_phases = (
                'FINAL_STEER_SETTLE',
                'FINAL_DRIVE',
                'FINAL_MEASURE_STOP',
                'FINAL_ZERO_STEER_SETTLE',
                'FINAL_STRAIGHT_DRIVE',
            )
            if self.reverse_phase in final_reverse_phases:
                if (
                    self.reference_lower_gone
                    or self.reference_upper_gone
                ):
                    self.transition(ParkingState.PARKED, now)
                    self.reverse_phase = 'PARKED_HOLD'
                    self.precision_reverse_guidance_source = 'PARKED'
                    self.publish_control(0, 0)
                    self.get_logger().info(
                        'PARKED: an original parked vehicle cleared the green '
                        'horizontal line; later pillars/units ignored'
                    )
                    return

                if self.reverse_phase == 'FINAL_STRAIGHT_DRIVE':
                    self.publish_control(0, self.reverse_speed)
                    return

                if self.reverse_phase == 'FINAL_ZERO_STEER_SETTLE':
                    self.publish_control(0, 0)
                    if phase_elapsed >= self.steer_settle_sec:
                        self.reverse_phase = 'FINAL_STRAIGHT_DRIVE'
                        self.reverse_phase_started_at = now
                        self.publish_control(0, self.reverse_speed)
                        self.get_logger().info(
                            'Steering centered at zero while stopped: '
                            'starting straight reverse'
                        )
                    return

                pair = self.observation.pair
                if (
                    pair is None
                    and self.current_reference_lower is not None
                    and self.current_reference_upper is not None
                ):
                    pair = self.build_parking_pair(
                        self.current_reference_lower,
                        self.current_reference_upper,
                        require_valid_gap=False,
                    )
                if pair is not None:
                    red_line_angle = self.reverse_reference_angle(pair)
                    lines_aligned = (
                        abs(red_line_angle)
                        <= self.final_line_alignment_tolerance
                    )
                    if lines_aligned:
                        calculated_steer = (
                            self.final_initial_alignment_steering(pair)
                        )
                        target_description = (
                            f'lines aligned ({red_line_angle:+.1f}deg): '
                            'average vehicle tilt='
                            f'{self.final_average_alignment_angle(pair):+.1f}'
                            f'deg x{self.final_reverse_steer_multiplier:.1f}'
                        )
                    else:
                        calculated_steer = self.reverse_steering(pair)
                        target_description = (
                            f'lines misaligned ({red_line_angle:+.1f}deg): '
                            f'line angle x{self.reverse_steer_multiplier:.1f}'
                        )
                    target_description += (
                        f', axes='
                        f'{math.degrees(pair.lower.axis_angle):+.0f}/'
                        f'{math.degrees(pair.upper.axis_angle):+.0f}deg'
                    )
                else:
                    self.precision_reverse_guidance_source = 'WAITING_SENSOR'
                    self.reverse_phase = 'FINAL_MEASURE_STOP'
                    self.reverse_phase_started_at = now
                    self.publish_control(self.held_steering_command(), 0)
                    return

                if self.reverse_phase == 'FINAL_STEER_SETTLE':
                    self.publish_control(self.reverse_segment_steer, 0)
                    if phase_elapsed >= self.steer_settle_sec:
                        self.reverse_phase = 'FINAL_DRIVE'
                        self.reverse_segment_started_at = now
                        self.get_logger().info(
                            f'Final reverse segment '
                            f'{self.reverse_segment_index}: '
                            f'steer={self.reverse_segment_steer}deg for '
                            f'{self.final_correction_duration:.1f}s'
                        )
                    return

                if self.reverse_phase == 'FINAL_DRIVE':
                    if (
                        self.reverse_segment_started_at is not None
                        and now - self.reverse_segment_started_at
                        >= self.final_correction_duration
                    ):
                        if (
                            self.final_correction_count
                            >= self.final_correction_segment_count
                        ):
                            self.reverse_phase = 'FINAL_ZERO_STEER_SETTLE'
                            self.reverse_phase_started_at = now
                            self.publish_control(0, 0)
                            self.get_logger().info(
                                f'{self.final_correction_segment_count} '
                                'LiDAR corrections complete: mandatory stop '
                                'and steer=0 centering before straight reverse'
                            )
                            return
                        self.reverse_phase = 'FINAL_MEASURE_STOP'
                        self.reverse_phase_started_at = now
                        self.publish_control(
                            self.held_steering_command(), 0
                        )
                        return
                    self.publish_control(
                        self.reverse_segment_steer,
                        self.reverse_speed,
                    )
                    return

                self.publish_control(self.held_steering_command(), 0)
                if phase_elapsed < self.reverse_measure_stop:
                    return

                self.reverse_segment_steer = calculated_steer
                self.final_correction_count += 1
                self.reverse_segment_index = self.final_correction_count
                self.reverse_phase = 'FINAL_STEER_SETTLE'
                self.reverse_phase_started_at = now
                self.get_logger().info(
                    f'Final reverse segment '
                    f'{self.final_correction_count}: '
                    f'{target_description}, '
                    f'steer={self.reverse_segment_steer}deg'
                )
                return

            if self.reverse_phase in lidar_reverse_phases:
                pair = self.observation.pair
                if pair is None:
                    self.reverse_phase = 'MEASURE_STOP'
                    self.reverse_phase_started_at = now
                    self.publish_control(self.held_steering_command(), 0)
                    return

                if self.reverse_phase == 'STEER_SETTLE':
                    self.publish_control(self.reverse_segment_steer, 0)
                    if (
                        self.reverse_phase_started_at is not None
                        and now - self.reverse_phase_started_at
                        >= self.steer_settle_sec
                    ):
                        self.reverse_segment_drive_duration = (
                            self.reverse_segment_duration
                        )
                        self.reverse_phase = 'DRIVE'
                        self.reverse_segment_started_at = now
                        self.get_logger().info(
                            f'LiDAR reverse segment '
                            f'{self.reverse_segment_index}: '
                            f'steer={self.reverse_segment_steer}deg for '
                            f'{self.reverse_segment_drive_duration:.1f}s'
                        )
                    return

                if self.reverse_phase == 'DRIVE':
                    if (
                        self.reverse_segment_started_at is not None
                        and now - self.reverse_segment_started_at
                        >= self.reverse_segment_drive_duration
                    ):
                        self.reverse_phase = 'MEASURE_STOP'
                        self.reverse_phase_started_at = now
                        self.publish_control(
                            self.held_steering_command(), 0
                        )
                        return
                    self.publish_control(
                        self.reverse_segment_steer,
                        self.reverse_speed,
                    )
                    return

                self.publish_control(self.held_steering_command(), 0)
                if (
                    self.reverse_phase_started_at is None
                    or now - self.reverse_phase_started_at
                    < self.reverse_measure_stop
                ):
                    return

                self.reverse_segment_index += 1
                self.reverse_segment_steer = (
                    self.pre_final_reverse_steering(pair)
                )
                target_description = (
                    f'reference=({pair.reference_point[0]:+.2f},'
                    f'{pair.reference_point[1]:+.2f})m, edge-tilt='
                    f'{self.final_average_alignment_angle(pair):+.1f}deg'
                )
                self.reverse_phase = 'STEER_SETTLE'
                self.reverse_phase_started_at = now
                self.get_logger().info(
                    f'LiDAR reverse segment '
                    f'{self.reverse_segment_index}: '
                    f'{target_description}, '
                    f'steer={self.reverse_segment_steer}deg'
                )
                return

            self.reverse_phase = 'MEASURE_STOP'
            self.reverse_phase_started_at = now
            self.lidar_side_far_frames = 0
            self.publish_control(0, 0)
            return

        if self.state == ParkingState.PARKED:
            self.reverse_phase = 'PARKED_HOLD_BEFORE_EXIT'
            self.publish_control(0, 0)
            if elapsed >= self.exit_wait_after_park:
                self.mode = ParkingMode.EXIT
                self.transition(ParkingState.EXIT_FORWARD, now)
                self.reverse_phase = 'EXIT_FORWARD'
                self.publish_control(0, self.exit_speed)
                self.get_logger().info(
                    f'EXIT: parked hold complete after '
                    f'{self.exit_wait_after_park:.1f}s; starting '
                    f'{self.exit_forward_duration:.1f}s straight drive'
                )
            return

        if self.state == ParkingState.EXIT_FORWARD:
            self.reverse_phase = 'EXIT_FORWARD'
            if elapsed >= self.exit_forward_duration:
                self.transition(ParkingState.EXIT_SET_RIGHT_STEER, now)
                self.reverse_phase = 'EXIT_SET_RIGHT_STEER'
                self.publish_control(self.exit_right_steer, 0)
                self.get_logger().info(
                    'EXIT: first straight complete; stopped to set maximum '
                    'right steering'
                )
                return
            self.publish_control(0, self.exit_speed)
            return

        if self.state == ParkingState.EXIT_SET_RIGHT_STEER:
            self.reverse_phase = 'EXIT_SET_RIGHT_STEER'
            if elapsed >= self.steer_settle_sec:
                self.transition(ParkingState.EXIT_RIGHT_TURN, now)
                self.reverse_phase = 'EXIT_RIGHT_TURN'
                self.publish_control(
                    self.exit_right_steer, self.exit_speed
                )
                self.get_logger().info(
                    f'EXIT: starting {self.exit_right_turn_duration:.1f}s '
                    'maximum-right turn'
                )
                return
            self.publish_control(self.exit_right_steer, 0)
            return

        if self.state == ParkingState.EXIT_RIGHT_TURN:
            self.reverse_phase = 'EXIT_RIGHT_TURN'
            if elapsed >= self.exit_right_turn_duration:
                self.transition(ParkingState.EXIT_CENTER_STEER, now)
                self.reverse_phase = 'EXIT_CENTER_STEER'
                self.publish_control(0, 0)
                self.get_logger().info(
                    'EXIT: right turn complete; stopped to center steering'
                )
                return
            self.publish_control(self.exit_right_steer, self.exit_speed)
            return

        if self.state == ParkingState.EXIT_CENTER_STEER:
            self.reverse_phase = 'EXIT_CENTER_STEER'
            if elapsed >= self.steer_settle_sec:
                self.transition(ParkingState.EXIT_FINAL_FORWARD, now)
                self.reverse_phase = 'EXIT_FINAL_FORWARD'
                self.publish_control(0, self.exit_speed)
                self.get_logger().info(
                    f'EXIT: starting final '
                    f'{self.exit_final_forward_duration:.1f}s straight drive'
                )
                return
            self.publish_control(0, 0)
            return

        if self.state == ParkingState.EXIT_FINAL_FORWARD:
            self.reverse_phase = 'EXIT_FINAL_FORWARD'
            if elapsed >= self.exit_final_forward_duration:
                self.transition(ParkingState.EXIT_COMPLETE, now)
                self.reverse_phase = 'EXIT_COMPLETE_HOLD'
                self.publish_control(0, 0)
                self.get_logger().info(
                    'EXIT COMPLETE: vehicle stopped with steering centered'
                )
                return
            self.publish_control(0, self.exit_speed)
            return

        # Other safe states hold their current angle to avoid hunting.
        self.publish_control(self.held_steering_command(), 0)

    def reverse_steering(self, pair: ParkingPair) -> int:
        """Return the red angle from the rear baseline to the reference point."""
        return self.scaled_reverse_steering(
            self.reverse_reference_angle(pair)
        )

    def pre_final_reverse_steering(self, pair: ParkingPair) -> int:
        """Combine strong midpoint centering with gentle edge alignment."""
        center_command = (
            self.reverse_reference_angle(pair)
            * self.reverse_steer_multiplier
        )
        alignment_command = (
            self.final_average_alignment_angle(pair)
            * self.pre_final_alignment_steer_multiplier
        )
        return int(round(max(
            -45.0,
            min(45.0, center_command + alignment_command),
        )))

    @staticmethod
    def reverse_reference_angle(pair: ParkingPair) -> float:
        """Return the unscaled red/green-line angular error in degrees."""
        reference_x = float(pair.reference_point[0])
        reference_y = float(pair.reference_point[1])
        # Rear baseline is -x. In the vehicle's LiDAR convention, a point to
        # the right has positive bearing and therefore commands right steer.
        rear_distance = max(0.05, -reference_x)
        return math.degrees(
            math.atan2(-reference_y, rear_distance)
        )

    def final_vehicle_gap_edge_angle(
        self,
        vehicle: VehicleCluster,
        *,
        is_lower: bool,
    ) -> Optional[tuple[float, float]]:
        """Fit one vehicle's gap-facing longitudinal edge.

        ``is_lower`` is the right-side vehicle in the debug/LiDAR view, so
        its upper y edge faces the gap. The left vehicle uses its lower edge.
        Returns ``(angle_degrees, x_span)`` only for a sufficiently long,
        longitudinal edge; short transverse box faces are intentionally
        rejected.
        """
        points = vehicle.points
        if len(points) < self.final_edge_min_bin_count * 2:
            return None

        x_low, x_high = np.quantile(points[:, 0], (0.05, 0.95))
        x_span = float(x_high - x_low)
        if x_span < self.final_edge_min_x_span:
            return None

        bin_edges = np.linspace(
            float(x_low), float(x_high), self.final_edge_bin_count + 1
        )
        edge_points: list[tuple[float, float]] = []
        edge_quantile = 0.85 if is_lower else 0.15
        for bin_low, bin_high in zip(bin_edges[:-1], bin_edges[1:]):
            in_bin = points[
                (points[:, 0] >= bin_low)
                & (points[:, 0] <= bin_high)
            ]
            if len(in_bin) < 2:
                continue
            edge_points.append((
                float(np.median(in_bin[:, 0])),
                float(np.quantile(in_bin[:, 1], edge_quantile)),
            ))

        if len(edge_points) < self.final_edge_min_bin_count:
            return None
        edge_array = np.asarray(edge_points, dtype=np.float64)
        fitted_slope = float(np.polyfit(
            edge_array[:, 0], edge_array[:, 1], 1
        )[0])
        angle = math.degrees(math.atan(fitted_slope))
        if abs(angle) > self.final_edge_max_abs_angle:
            return None
        return angle, x_span

    def final_average_alignment_angle(self, pair: ParkingPair) -> float:
        """Return a span-weighted mean of reliable longitudinal edges."""
        estimates = [
            self.final_vehicle_gap_edge_angle(pair.lower, is_lower=True),
            self.final_vehicle_gap_edge_angle(pair.upper, is_lower=False),
        ]
        valid_estimates = [item for item in estimates if item is not None]
        if not valid_estimates:
            return 0.0
        total_span = sum(item[1] for item in valid_estimates)
        if total_span <= 0.0:
            return 0.0
        return sum(
            angle * span for angle, span in valid_estimates
        ) / total_span

    def final_initial_alignment_steering(
        self, pair: ParkingPair
    ) -> int:
        """Scale only the two parked vehicles' mean tilt."""
        return self.scaled_reverse_steering(
            self.final_average_alignment_angle(pair),
            multiplier=self.final_reverse_steer_multiplier,
        )

    def final_conditional_steering(self, pair: ParkingPair) -> int:
        """Choose line correction unless red and green are already aligned."""
        if (
            abs(self.reverse_reference_angle(pair))
            <= self.final_line_alignment_tolerance
        ):
            return self.final_initial_alignment_steering(pair)
        return self.reverse_steering(pair)

    def final_single_reference_steering(
        self,
        vehicle: VehicleCluster,
        *,
        is_lower: bool,
    ) -> int:
        """Continue after one original vehicle clears, without using a pillar."""
        inferred_gap_center = (
            float(vehicle.y_max) + self.final_target_half_gap
            if is_lower
            else float(vehicle.y_min) - self.final_target_half_gap
        )
        reference_depth = max(0.05, -float(vehicle.center[0]))
        center_angle = math.degrees(math.atan2(
            -inferred_gap_center,
            reference_depth,
        ))
        alignment_angle = math.degrees(float(vehicle.axis_angle))
        calculated_angle = (
            self.final_center_gain * center_angle
            + self.final_alignment_gain * alignment_angle
        )
        return self.scaled_reverse_steering(
            calculated_angle,
            multiplier=self.final_reverse_steer_multiplier,
        )

    def single_vehicle_steering(self, vehicle: VehicleCluster) -> int:
        """Parallelize the ego with the only visible parked-vehicle line."""
        axis_angle = math.degrees(vehicle.axis_angle)
        if abs(axis_angle) <= self.single_vehicle_angle_deadband:
            return 0
        # In this LiDAR/debug convention +axis tilts toward the right while
        # reversing, so it directly produces a positive (right) steer.
        return self.scaled_reverse_steering(axis_angle)

    def scaled_reverse_steering(
        self,
        calculated_angle: float,
        *,
        multiplier: Optional[float] = None,
    ) -> int:
        """Scale a LiDAR reverse angle and clamp it to the steering range."""
        steer_multiplier = (
            self.reverse_steer_multiplier
            if multiplier is None
            else multiplier
        )
        scaled_angle = calculated_angle * steer_multiplier
        return int(round(max(-45.0, min(45.0, scaled_angle))))

    @staticmethod
    def vehicle_nearest_distance(vehicle: VehicleCluster) -> float:
        """Distance to the nearest LiDAR point belonging to one vehicle."""
        if len(vehicle.points) == 0:
            return math.inf
        return float(np.min(np.linalg.norm(vehicle.points, axis=1)))

    def fail(self, reason: str, now: float) -> None:
        self.failure_reason = reason
        self.get_logger().error(f'Parking failed: {reason}')
        self.transition(ParkingState.PARKING_FAILED, now)
        self.publish_control(self.held_steering_command(), 0)

    def shutdown_program(self) -> None:
        """Stop this node and its ros2 launch parent after recognition."""
        if self.shutdown_started:
            return
        self.shutdown_started = True
        self.publish_control(0, 0)
        self.get_logger().info(
            'Recognition-only test complete; shutting down parking launch'
        )

        # rclpy.shutdown() stops only this process. When this node was started
        # through ``ros2 launch``, also notify that exact parent so its sensor
        # and motor processes are cleaned up. Never signal an unrelated parent.
        parent_pid = os.getppid()
        try:
            with open(
                f'/proc/{parent_pid}/cmdline',
                'rb',
            ) as command_file:
                parent_command = command_file.read().replace(b'\x00', b' ')
            if b'ros2' in parent_command and b'launch' in parent_command:
                os.kill(parent_pid, signal.SIGINT)
        except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
            pass

        if rclpy.ok():
            rclpy.shutdown()

    def publish_control(self, steer: int, speed: int) -> None:
        steer = int(max(-45, min(45, steer)))
        speed = int(max(-130, min(130, speed)))
        self.last_command = (steer, speed)
        message = Int16MultiArray()
        message.data = [steer, speed]
        self.motor_publisher.publish(message)

    def draw_debug(self) -> None:
        size = self.debug_image_size
        center = size // 2
        scale = size * 0.42 / self.debug_max_range
        image = np.zeros((size, size, 3), dtype=np.uint8)

        for radius_m in np.arange(0.5, self.debug_max_range + 0.01, 0.5):
            radius = int(radius_m * scale)
            cv2.circle(image, (center, center), radius, (0, 60, 0), 1)
        cv2.line(image, (center, 20), (center, size - 20), (0, 70, 0), 1)
        cv2.line(image, (20, center), (size - 20, center), (0, 70, 0), 1)

        def pixel(point: np.ndarray | tuple[float, float]):
            x_forward, y_left = float(point[0]), float(point[1])
            return (
                int(center - y_left * scale),
                int(center - x_forward * scale),
            )

        for point in self.observation.points:
            cv2.circle(image, pixel(point), 1, (100, 100, 100), -1)

        colors = [
            (0, 220, 255), (255, 150, 0), (180, 80, 255), (60, 200, 60)
        ]
        for index, vehicle in enumerate(self.observation.vehicles):
            color = colors[index % len(colors)]
            for point in vehicle.points:
                cv2.circle(image, pixel(point), 2, color, -1)
            location = pixel(vehicle.center)
            cv2.putText(
                image,
                f'V{index + 1} {math.degrees(vehicle.axis_angle):+.0f}deg',
                (location[0] + 5, location[1] - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA,
            )

        pair = self.observation.pair
        if pair is not None:
            target_y = pair.gap_center_y
            line_start = pixel((-self.debug_max_range, target_y))
            line_end = pixel((self.debug_max_range, target_y))
            cv2.line(image, line_start, line_end, (0, 255, 0), 2)
            reference_pixel = pixel(pair.reference_point)
            rear_baseline_end = pixel(
                (-min(1.8, self.debug_max_range), 0.0)
            )
            cv2.line(
                image,
                (center, center),
                rear_baseline_end,
                (0, 0, 255),
                3,
            )
            cv2.line(
                image,
                (center, center),
                reference_pixel,
                (0, 0, 255),
                3,
            )
            cv2.circle(image, reference_pixel, 9, (0, 0, 255), -1)

        # Vehicle marker and LiDAR-to-rear-bumper reference.
        cv2.circle(image, (center, center), 7, (255, 255, 255), -1)
        bumper_left = pixel((-self.lidar_to_rear_bumper, -self.vehicle_width / 2))
        bumper_right = pixel((-self.lidar_to_rear_bumper, self.vehicle_width / 2))
        cv2.line(image, bumper_left, bumper_right, (255, 255, 255), 3)

        cv2.rectangle(image, (8, 8), (size - 8, 132), (55, 55, 55), -1)
        if self.state in (
            ParkingState.PARKED,
            ParkingState.EXIT_COMPLETE,
        ):
            state_text = self.state.value
            state_color = (0, 255, 0)
        elif self.state in (
            ParkingState.PARKING_FAILED,
            ParkingState.EMERGENCY_STOP,
        ):
            state_text = 'FAILED'
            state_color = (0, 0, 255)
        else:
            state_text = self.state.value
            state_color = (0, 255, 255)
        cv2.putText(
            image,
            f'MODE: {self.mode.value} | STATE: {state_text}',
            (18, 36),
            cv2.FONT_HERSHEY_SIMPLEX, 0.70, state_color, 2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f'cmd steer/speed={self.last_command[0]}/{self.last_command[1]} '
            f'phase={self.reverse_phase} via=/motor_control',
            (18, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.50,
            (255, 255, 255), 1, cv2.LINE_AA,
        )
        if pair is None:
            if len(self.observation.vehicles) == 1:
                pair_text = (
                    'one vehicle visible: waiting for two-vehicle midpoint '
                    f'| segment={self.reverse_segment_index} '
                    f'rear-lidar={self.rear_half_lidar_point_count} '
                    f'empty={self.rear_half_empty_frames}/'
                    f'{self.rear_half_empty_confirm_frames}'
                )
            else:
                pair_text = (
                    'gap: not acquired '
                    f'| vehicles={len(self.observation.vehicles)} '
                    f'segment={self.reverse_segment_index} '
                    f'rear-lidar={self.rear_half_lidar_point_count} '
                    f'empty={self.rear_half_empty_frames}/'
                    f'{self.rear_half_empty_confirm_frames}'
                )
        else:
            displayed_steer = (
                (
                    0
                    if self.reverse_phase in (
                        'FINAL_ZERO_STEER_SETTLE',
                        'FINAL_STRAIGHT_DRIVE',
                    )
                    else self.reverse_segment_steer
                )
                if self.reverse_phase.startswith('FINAL_')
                else self.pre_final_reverse_steering(pair)
            )
            pair_text = (
                f'pair='
                f'{"FALLBACK" if self.observation.pair_is_fallback else "STRICT"} '
                f'seg={self.reverse_segment_index} '
                f'gap={pair.gap_width:.2f}m center='
                f'{pair.gap_center_y:+.2f}m L/R='
                f'{pair.left_clearance:.2f}/'
                f'{pair.right_clearance:.2f}m '
                f'ref-angle={displayed_steer:+d}deg '
                f'rear-lidar={self.rear_half_lidar_point_count} '
                f'empty={self.rear_half_empty_frames}/'
                f'{self.rear_half_empty_confirm_frames}'
            )
        cv2.putText(
            image, pair_text, (18, 96),
            cv2.FONT_HERSHEY_SIMPLEX, 0.47, (255, 255, 255), 1,
            cv2.LINE_AA,
        )
        if self.failure_reason:
            cv2.putText(
                image,
                f'failure: {self.failure_reason}',
                (18, 121),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.43,
                (80, 80, 255),
                1,
                cv2.LINE_AA,
            )
        elif self.state == ParkingState.REVERSE_CENTER:
            if self.reverse_phase in (
                'STEER_SETTLE',
                'DRIVE',
                'MEASURE_STOP',
            ):
                guidance_text = (
                    f'1s midpoint correction | nearest vehicle='
                    f'{self.straight_reverse_trigger_distance:.2f}m '
                    f'gate={self.straight_reverse_radius:.2f}m'
                )
            elif self.reverse_phase == 'FINAL_STOP':
                remaining = max(
                    0.0,
                    self.straight_reverse_stop
                    - (
                        0.0
                        if self.reverse_phase_started_at is None
                        else time.monotonic()
                        - self.reverse_phase_started_at
                    ),
                )
                guidance_text = (
                    f'1m LATCHED | STOP + STEER 0 | '
                    f'{remaining:.1f}s remaining'
                )
            elif self.reverse_phase in (
                'FINAL_STEER_SETTLE',
                'FINAL_DRIVE',
                'FINAL_MEASURE_STOP',
                'FINAL_ZERO_STEER_SETTLE',
                'FINAL_STRAIGHT_DRIVE',
            ):
                if self.reverse_phase == 'FINAL_ZERO_STEER_SETTLE':
                    guidance_text = (
                        'MANDATORY STOP | centering steer=0 before reverse'
                    )
                elif self.reverse_phase == 'FINAL_STRAIGHT_DRIVE':
                    guidance_text = (
                        'FINAL steer=0 continuous reverse | '
                        f'original gone L/R='
                        f'{int(self.reference_upper_gone)}/'
                        f'{int(self.reference_lower_gone)}'
                    )
                else:
                    guidance_text = (
                        f'{self.final_correction_duration:.1f}s LiDAR '
                        f'correction {self.final_correction_count}/'
                        f'{self.final_correction_segment_count} | '
                        f'original gone L/R='
                        f'{int(self.reference_upper_gone)}/'
                        f'{int(self.reference_lower_gone)}'
                    )
            else:
                guidance_text = f'phase={self.reverse_phase}'
            cv2.putText(
                image,
                guidance_text,
                (18, 121),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.43,
                (120, 255, 120),
                1,
                cv2.LINE_AA,
            )
        cv2.putText(
            image,
            'REAR 0 | RIGHT +90 | FRONT +/-180 | LEFT -90',
            (18, size - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
            (0, 220, 220), 1, cv2.LINE_AA,
        )
        cv2.imshow(self.debug_window_name, image)
        if self.camera_debug_image is not None:
            camera_display = self.camera_debug_image.copy()
            camera_active = (
                self.precision_reverse_guidance_source == 'CAMERA'
            )
            banner = (
                'CAMERA STEERING ACTIVE'
                if camera_active
                else 'CAMERA MONITOR ONLY | control='
                f'{self.precision_reverse_guidance_source}'
            )
            banner_color = (
                (0, 255, 0) if camera_active else (0, 180, 255)
            )
            border_thickness = 8 if camera_active else 3
            cv2.rectangle(
                camera_display,
                (1, 1),
                (
                    camera_display.shape[1] - 2,
                    camera_display.shape[0] - 2,
                ),
                banner_color,
                border_thickness,
            )
            cv2.putText(
                camera_display,
                banner,
                (12, 48),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.82,
                (0, 0, 0),
                5,
                cv2.LINE_AA,
            )
            cv2.putText(
                camera_display,
                banner,
                (12, 48),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.82,
                banner_color,
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(
                self.camera_debug_window_name,
                camera_display,
            )
        cv2.waitKey(1)

    def destroy_node(self):
        try:
            self.publish_control(0, 0)
            if self.debug_view:
                cv2.destroyWindow(self.debug_window_name)
                if self.camera_debug_image is not None:
                    cv2.destroyWindow(self.camera_debug_window_name)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingNodeYym()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
