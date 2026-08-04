"""LiDAR-only parking controller: /scan -> /motor_control.

Scan convention:
    0 deg = vehicle rear, +90 deg = right, -90 deg = left,
    and +/-180 deg = front. Angles increase counter-clockwise.
The parking-valid field is rear-centered and split by sign into RIGHT and LEFT.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16MultiArray


class ParkingState(str, Enum):
    WAIT_FOR_SCAN = 'WAIT_FOR_SCAN'
    WAIT_RIGHT_CLEAR_INITIAL = 'WAIT_RIGHT_CLEAR_INITIAL'
    WAIT_CAR1_ENTRY = 'WAIT_CAR1_ENTRY'
    WAIT_CAR2_ENTRY = 'WAIT_CAR2_ENTRY'
    PASS_SECOND_CAR = 'PASS_SECOND_CAR'
    SET_REVERSE_STEER = 'SET_REVERSE_STEER'
    REVERSE_HARD_RIGHT = 'REVERSE_HARD_RIGHT'
    PARK_STOP = 'PARK_STOP'
    EXIT_SET_RIGHT_STEER = 'EXIT_SET_RIGHT_STEER'
    EXIT_RIGHT_TURN = 'EXIT_RIGHT_TURN'
    EXIT_FORWARD = 'EXIT_FORWARD'
    DONE = 'DONE'
    PARKING_FAILED = 'PARKING_FAILED'
    EMERGENCY_STOP = 'EMERGENCY_STOP'


@dataclass
class RearObservation:
    """Rear LiDAR clusters plus the two parked-car bundles used for alignment."""

    clusters: list[np.ndarray]
    right_clusters: list[np.ndarray]
    vehicle_bundles: list[np.ndarray]
    right_vehicle_bundles: list[np.ndarray]
    left_point_count: int
    right_point_count: int
    scan_valid: bool
    rear_axis_min_distance: Optional[float]
    bundle1: Optional[np.ndarray]
    bundle2: Optional[np.ndarray]
    bundle1_visible: bool
    bundle2_visible: bool
    bundle1_distance: Optional[float]
    bundle2_distance: Optional[float]

    @property
    def two_vehicle_bundles_visible(self) -> bool:
        return self.bundle1_visible and self.bundle2_visible

    @property
    def vehicle_bundle_count(self) -> int:
        return len(self.vehicle_bundles)

    @property
    def right_visible(self) -> bool:
        return bool(self.right_vehicle_bundles)

    @property
    def right_bundle_count(self) -> int:
        return len(self.right_vehicle_bundles)


class ParkingNodeOsy(Node):
    """Hard-coded T-parking sequence with LEFT/RIGHT LiDAR fine alignment."""

    def __init__(self) -> None:
        super().__init__('parking_node_osy')

        # This node deliberately has only one input and one output.
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('motor_topic', '/motor_control')
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('scan_timeout_sec', 0.5)
        self.declare_parameter('scan_quality_min_points', 10)
        self.declare_parameter('invalid_scan_confirm_frames', 5)
        self.declare_parameter('debug_view', True)
        self.declare_parameter('debug_window_name', 'parking_lidar_sequence_debug')
        self.declare_parameter('debug_hz', 30.0)

        # Same physical field as the old |front-zero angle| >= 70 degrees.
        # In the rear-zero convention it is LEFT -110..0, RIGHT 0..+110.
        self.declare_parameter('valid_sector_max_abs_deg', 110.0)
        self.declare_parameter('rear_hard_stop_angle_deg', 12.0)
        self.declare_parameter('rear_hard_stop_distance_m', 0.18)
        # Ignore LiDAR returns closer than the sensor/vehicle blind zone.
        self.declare_parameter('parking_min_range_m', 0.15)
        # 2.0m 안의 차량 흔적만 주차 판단에 사용한다.
        self.declare_parameter('cluster_max_range_m', 2.5)
        self.declare_parameter('cluster_join_distance_m', 0.5)
        self.declare_parameter('cluster_min_points', 5)
        # Region-growing radius: points close to any point already in a car
        # bundle are recursively absorbed, so elongated/elliptic traces stay
        # as one bundle.
        self.declare_parameter('vehicle_bundle_neighbor_distance_m', 0.22)
        # A handful of isolated reflections must not become B1/B2.  The bag
        # contains 10~19 point fragments around the actual vehicle traces.
        self.declare_parameter('vehicle_bundle_min_points', 10)
        # Ignore one valid side completely unless it has this many returns.
        self.declare_parameter('valid_side_min_points', 30)
        self.declare_parameter('right_empty_confirm_sec', 0.5)
        self.declare_parameter('right_entry_confirm_frames', 3)
        self.declare_parameter('reverse_pair_confirm_frames', 3)
        self.declare_parameter('reverse_b2_turnaround_epsilon_m', 0.01)
        self.declare_parameter('reverse_b2_turnaround_max_distance_m', 0.9)
        self.declare_parameter('b2_left_track_max_jump_m', 1.80)
        self.declare_parameter('bundle_track_max_jump_m', 0.65)

        # Motion signs are hardware dependent.  Per request, entry/exit turn
        # defaults are right turn; tune these values on the vehicle.
        self.declare_parameter('forward_speed', 110)
        self.declare_parameter('reverse_speed', -100)
        self.declare_parameter('right_turn_steer', 45)
        self.declare_parameter('pre_reverse_left_steer', -45)
        self.declare_parameter('steer_settle_sec', 0.60)
        self.declare_parameter('pass_second_car_sec', 2.5)
        self.declare_parameter('approach_timeout_sec', 30.0)
        self.declare_parameter('reverse_seek_timeout_sec', 10.0)
        self.declare_parameter('park_stop_sec', 1.0)
        self.declare_parameter('exit_right_turn_sec', 10.0)
        self.declare_parameter('exit_forward_sec', 8.0)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.motor_topic = str(self.get_parameter('motor_topic').value)
        self.control_hz = max(1.0, float(self.get_parameter('control_hz').value))
        self.scan_timeout_sec = float(self.get_parameter('scan_timeout_sec').value)
        self.scan_quality_min_points = int(
            self.get_parameter('scan_quality_min_points').value
        )
        self.invalid_scan_confirm_frames = int(
            self.get_parameter('invalid_scan_confirm_frames').value
        )
        self.debug_view = bool(self.get_parameter('debug_view').value)
        self.debug_window_name = str(self.get_parameter('debug_window_name').value)
        self.debug_hz = max(1.0, float(self.get_parameter('debug_hz').value))
        self.valid_sector_max_abs = math.radians(float(
            self.get_parameter('valid_sector_max_abs_deg').value
        ))
        self.rear_hard_stop_angle = math.radians(float(
            self.get_parameter('rear_hard_stop_angle_deg').value
        ))
        self.rear_hard_stop_distance = float(
            self.get_parameter('rear_hard_stop_distance_m').value
        )
        self.cluster_max_range = float(self.get_parameter('cluster_max_range_m').value)
        self.parking_min_range = float(max(
            0.0, self.get_parameter('parking_min_range_m').value
        ))
        self.cluster_join_distance = float(
            self.get_parameter('cluster_join_distance_m').value
        )
        self.cluster_min_points = int(self.get_parameter('cluster_min_points').value)
        self.vehicle_bundle_neighbor_distance = float(
            self.get_parameter('vehicle_bundle_neighbor_distance_m').value
        )
        self.vehicle_bundle_min_points = int(
            self.get_parameter('vehicle_bundle_min_points').value
        )
        self.valid_side_min_points = int(
            self.get_parameter('valid_side_min_points').value
        )
        self.right_empty_confirm_sec = float(
            self.get_parameter('right_empty_confirm_sec').value
        )
        self.right_entry_confirm_frames = int(
            self.get_parameter('right_entry_confirm_frames').value
        )
        self.reverse_pair_confirm_frames = int(
            self.get_parameter('reverse_pair_confirm_frames').value
        )
        self.reverse_b2_turnaround_epsilon = float(max(
            0.0, self.get_parameter('reverse_b2_turnaround_epsilon_m').value
        ))
        self.reverse_b2_turnaround_max_distance = float(max(
            0.0,
            self.get_parameter('reverse_b2_turnaround_max_distance_m').value,
        ))
        self.b2_left_track_max_jump = float(
            self.get_parameter('b2_left_track_max_jump_m').value
        )
        self.bundle_track_max_jump = float(
            self.get_parameter('bundle_track_max_jump_m').value
        )
        self.forward_speed = int(self.get_parameter('forward_speed').value)
        self.reverse_speed = int(self.get_parameter('reverse_speed').value)
        self.right_turn_steer = int(self.get_parameter('right_turn_steer').value)
        self.pre_reverse_left_steer = int(
            self.get_parameter('pre_reverse_left_steer').value
        )
        self.steer_settle_sec = float(self.get_parameter('steer_settle_sec').value)
        self.pass_second_car_sec = float(
            self.get_parameter('pass_second_car_sec').value
        )
        self.approach_timeout_sec = float(
            self.get_parameter('approach_timeout_sec').value
        )
        self.reverse_seek_timeout_sec = float(
            self.get_parameter('reverse_seek_timeout_sec').value
        )
        self.park_stop_sec = float(self.get_parameter('park_stop_sec').value)
        self.exit_right_turn_sec = float(
            self.get_parameter('exit_right_turn_sec').value
        )
        self.exit_forward_sec = float(self.get_parameter('exit_forward_sec').value)

        # LiDAR 드라이버가 기동되어 첫 /scan을 내보낼 때까지는 emergency가 아닌
        # 안전 정지 대기 상태로 유지한다.
        self.state = ParkingState.WAIT_FOR_SCAN
        self.state_started_at = time.monotonic()
        self.last_scan_at: Optional[float] = None
        self.observation = RearObservation(
            [], [], [], [], 0, 0, False, None,
            None, None, False, False, None, None,
        )
        self.invalid_scan_count = 0
        self.approach_started_at: Optional[float] = None
        self.right_empty_since: Optional[float] = None
        self.right_entry_frames = 0
        self.right_two_bundles_seen = False
        self.reverse_pair_confirm_count = 0
        self.reverse_pair_confirmed = False
        self.reverse_b2_min_distance: Optional[float] = None
        self.reverse_b2_has_approached = False
        self.reverse_b2_turnaround_detected = False
        self.bundle_track_centroids: list[Optional[np.ndarray]] = [None, None]
        self.bundle_track_distances: list[Optional[float]] = [None, None]
        self.reverse_primary_seed_centroid: Optional[np.ndarray] = None
        self.reverse_primary_seed_distance: Optional[float] = None
        self.last_command = (0, 0)

        self.motor_pub = self.create_publisher(Int16MultiArray, self.motor_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_timer(1.0 / self.control_hz, self.control_tick)
        if self.debug_view:
            # Render at 30 Hz so the state/command overlay reacts promptly;
            # scan geometry itself updates whenever the LiDAR publishes.
            self.create_timer(1.0 / self.debug_hz, self.draw_debug)
        self.get_logger().info(
            'parking_node_osy: LiDAR-only /scan -> /motor_control; '
            'rear-zero convention, using |angle| <= 110 deg split into LEFT/RIGHT; '
            'car1/car2 are counted in the RIGHT valid side'
        )

    def scan_callback(self, msg: LaserScan) -> None:
        now = time.monotonic()
        self.last_scan_at = now
        observation = self.observe_rear_car_bundles(msg)
        if not observation.scan_valid:
            self.invalid_scan_count += 1
            return
        self.invalid_scan_count = 0
        self.observation = observation

        if self.state == ParkingState.REVERSE_HARD_RIGHT:
            self.update_bundle_tracks(self.observation.vehicle_bundles)
        else:
            self.clear_observation_tracks()

        # Count only distinct LiDAR scans, never repeated control ticks.  The
        # RIGHT bundles have already passed the per-side 30-point filter.
        right_count = self.observation.right_bundle_count
        if self.state == ParkingState.WAIT_CAR1_ENTRY:
            # First single parked-car bundle: begin waiting for car 2.
            self.right_entry_frames = (
                self.right_entry_frames + 1 if right_count == 1 else 0
            )
        elif self.state == ParkingState.WAIT_CAR2_ENTRY:
            if not self.right_two_bundles_seen:
                # Car 2 arrives while car 1 is still visible: two bundles.
                self.right_entry_frames = (
                    self.right_entry_frames + 1 if right_count >= 2 else 0
                )
            else:
                # Once the pair was seen, car 2 passing leaves one bundle;
                # this is the trigger for the hard-coded reverse entry.
                self.right_entry_frames = (
                    self.right_entry_frames + 1 if right_count == 1 else 0
                )
        else:
            self.right_entry_frames = 0

        if self.state == ParkingState.REVERSE_HARD_RIGHT:
            if not self.reverse_pair_confirmed:
                self.reverse_pair_confirm_count = (
                    self.reverse_pair_confirm_count + 1
                    if self.observation.two_vehicle_bundles_visible else 0
                )
                if self.reverse_pair_confirm_count >= self.reverse_pair_confirm_frames:
                    self.reverse_pair_confirmed = True
                    self.reverse_b2_min_distance = None
                    self.reverse_b2_has_approached = False
                    self.reverse_b2_turnaround_detected = False
                    self.get_logger().info(
                        'Reverse B1/B2 acquired on RIGHT; waiting for B2 turnaround'
                    )
            else:
                bundle2 = self.observation.bundle2
                if bundle2 is not None:
                    distance = self.bundle_distance(bundle2)
                    if self.reverse_b2_min_distance is None:
                        self.reverse_b2_min_distance = distance
                    elif distance < (
                        self.reverse_b2_min_distance
                        - self.reverse_b2_turnaround_epsilon
                    ):
                        self.reverse_b2_min_distance = distance
                        self.reverse_b2_has_approached = (
                            distance <= self.reverse_b2_turnaround_max_distance
                        )
                    elif (self.reverse_b2_has_approached
                            and self.reverse_b2_min_distance
                            <= self.reverse_b2_turnaround_max_distance
                            and distance > (
                                self.reverse_b2_min_distance
                                + self.reverse_b2_turnaround_epsilon
                            )):
                        self.reverse_b2_turnaround_detected = True
        else:
            self.reverse_pair_confirm_count = 0
            self.reverse_b2_min_distance = None
            self.reverse_b2_has_approached = False
            self.reverse_b2_turnaround_detected = False

    def observe_rear_car_bundles(self, msg: LaserScan) -> RearObservation:
        """Cluster the rear-centered field, split into LEFT and RIGHT."""
        left_ordered_points: list[tuple[float, float]] = []
        right_ordered_points: list[tuple[float, float]] = []
        valid_scan_points = 0
        rear_axis_distances: list[float] = []
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            if distance < self.parking_min_range:
                continue
            valid_scan_points += 1
            if distance > self.cluster_max_range:
                continue
            angle = msg.angle_min + index * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= self.rear_hard_stop_angle:
                rear_axis_distances.append(float(distance))
            if abs(angle) > self.valid_sector_max_abs:
                continue  # discard the front-centered invalid field
            point = (-distance * math.cos(angle), -distance * math.sin(angle))
            # Positive angles are RIGHT and drive the sequential car1/car2
            # approach events. Negative angles are LEFT.
            if angle > 0.0:
                right_ordered_points.append(point)
            else:
                left_ordered_points.append(point)

        left_point_count = len(left_ordered_points)
        right_point_count = len(right_ordered_points)
        rear_axis_min_distance = (
            min(rear_axis_distances) if rear_axis_distances else None
        )

        # Sparse returns are noise for parking purposes. LEFT and RIGHT must
        # independently contain at least 30 valid points before use.
        if len(left_ordered_points) < self.valid_side_min_points:
            left_ordered_points = []
        if len(right_ordered_points) < self.valid_side_min_points:
            right_ordered_points = []

        clusters = (
            self.cluster_ordered_points(left_ordered_points)
            + self.cluster_ordered_points(right_ordered_points)
        )
        right_clusters = self.cluster_ordered_points(right_ordered_points)
        vehicle_bundles = self.make_vehicle_bundles(clusters)
        right_vehicle_bundles = self.make_vehicle_bundles(right_clusters)
        return RearObservation(
            clusters, right_clusters, vehicle_bundles, right_vehicle_bundles,
            left_point_count, right_point_count,
            valid_scan_points >= self.scan_quality_min_points,
            rear_axis_min_distance,
            None, None, False, False, None, None,
        )

    def cluster_ordered_points(self, ordered_points: list[tuple[float, float]]) -> list[np.ndarray]:
        """Split consecutive scan returns into parked-car point bundles."""
        clusters: list[np.ndarray] = []
        current: list[tuple[float, float]] = []
        previous: Optional[np.ndarray] = None
        for point_tuple in ordered_points:
            point = np.asarray(point_tuple, dtype=np.float64)
            if previous is not None and np.linalg.norm(point - previous) > self.cluster_join_distance:
                if len(current) >= self.cluster_min_points:
                    clusters.append(np.asarray(current, dtype=np.float64))
                current = []
            current.append(point_tuple)
            previous = point
        if len(current) >= self.cluster_min_points:
            clusters.append(np.asarray(current, dtype=np.float64))
        return clusters

    def make_vehicle_bundles(self, clusters: list[np.ndarray]) -> list[np.ndarray]:
        """Region-grow nearby rear points, then select the two parked cars.

        Every point near an existing bundle point is recursively added.  This
        keeps a long or elliptic vehicle trace together even when its radial
        distance varies across the surface.  Separated vehicles remain split
        because no chain of nearby points connects them.
        """
        if not clusters:
            return []
        points = np.vstack(clusters)
        bundles: list[np.ndarray] = []
        unassigned = set(range(len(points)))
        while unassigned:
            seed = unassigned.pop()
            component = [seed]
            pending = [seed]
            while pending:
                current = pending.pop()
                candidates = np.fromiter(unassigned, dtype=np.intp)
                if len(candidates) == 0:
                    continue
                distances = np.linalg.norm(points[candidates] - points[current], axis=1)
                nearby = candidates[distances <= self.vehicle_bundle_neighbor_distance]
                for neighbor in nearby:
                    unassigned.remove(int(neighbor))
                    component.append(int(neighbor))
                    pending.append(int(neighbor))
            if len(component) >= self.vehicle_bundle_min_points:
                bundles.append(points[component])
        if not bundles:
            return []
        # Return every valid component.  Tracking chooses the physical car 1
        # and car 2 without hiding the important one-bundle/zero-bundle cases.
        return sorted(bundles, key=len, reverse=True)

    def bundle_distance(self, bundle: np.ndarray) -> float:
        """Euclidean distance from LiDAR origin to the bundle center."""
        center = self.bundle_centroid(bundle)
        return float(np.linalg.norm(center))

    @staticmethod
    def bundle_centroid(bundle: np.ndarray) -> np.ndarray:
        return np.median(bundle, axis=0)

    def reset_bundle_tracking(self) -> None:
        self.bundle_track_centroids = [None, None]
        self.bundle_track_distances = [None, None]
        self.reverse_pair_confirm_count = 0
        self.reverse_pair_confirmed = False
        self.reverse_b2_min_distance = None
        self.reverse_b2_has_approached = False
        self.reverse_b2_turnaround_detected = False
        self.clear_observation_tracks()

    def capture_reverse_primary_seed(self) -> None:
        """Remember the one RIGHT bundle left after the second-car pass."""
        if not self.observation.right_vehicle_bundles:
            self.reverse_primary_seed_centroid = None
            self.reverse_primary_seed_distance = None
            self.get_logger().warn('No RIGHT bundle available to seed reverse tracking')
            return
        primary = max(self.observation.right_vehicle_bundles, key=len)
        self.reverse_primary_seed_centroid = self.bundle_centroid(primary)
        self.reverse_primary_seed_distance = self.bundle_distance(primary)
        self.get_logger().info(
            'Reverse primary bundle seeded from RIGHT side: '
            f'{self.reverse_primary_seed_distance:.2f} m'
        )

    def clear_observation_tracks(self) -> None:
        self.observation.bundle1 = None
        self.observation.bundle2 = None
        self.observation.bundle1_visible = False
        self.observation.bundle2_visible = False
        self.observation.bundle1_distance = self.bundle_track_distances[0]
        self.observation.bundle2_distance = self.bundle_track_distances[1]

    def update_bundle_tracks(self, bundles: list[np.ndarray]) -> None:
        """Track the seeded RIGHT bundle first, then the newly visible bundle."""
        self.clear_observation_tracks()
        candidates = sorted(bundles, key=len, reverse=True)[:4]
        if not candidates:
            return
        centroids = [self.bundle_centroid(bundle) for bundle in candidates]

        assignments: dict[int, int] = {}
        if self.bundle_track_centroids[0] is None:
            # B1 must begin as the RIGHT-side bundle that was present before
            # reversing.  Do not initialise from a newly appearing left-side
            # bundle, otherwise car IDs can swap at reverse entry.
            right_indices = [
                index for index, centroid in enumerate(centroids)
                if float(centroid[1]) < 0.0
            ]
            if not right_indices:
                return
            if self.reverse_primary_seed_centroid is None:
                primary_index = right_indices[0]
            else:
                primary_index = min(
                    right_indices,
                    key=lambda index: float(np.linalg.norm(
                        centroids[index] - self.reverse_primary_seed_centroid
                    )),
                )
            assignments[0] = primary_index
            # If a distinct RIGHT bundle is already visible, register it as
            # B2; otherwise B1 remains tracked until it appears on RIGHT.
            secondary_indices = [
                index for index, centroid in enumerate(centroids)
                if index != primary_index and float(centroid[1]) < 0.0
            ]
            if secondary_indices:
                assignments[1] = secondary_indices[0]
        elif self.bundle_track_centroids[1] is None:
            # Keep B1 on the nearest continuation, then use the new distinct
            # RIGHT-side component as B2 when it enters the valid field.
            primary_index = min(
                range(len(candidates)),
                key=lambda index: float(np.linalg.norm(
                    centroids[index] - self.bundle_track_centroids[0]
                )),
            )
            primary_jump = float(np.linalg.norm(
                centroids[primary_index] - self.bundle_track_centroids[0]
            ))
            if primary_jump <= self.bundle_track_max_jump:
                assignments[0] = primary_index
                secondary_indices = [
                    index for index, centroid in enumerate(centroids)
                    if index != primary_index and float(centroid[1]) < 0.0
                ]
                if secondary_indices:
                    assignments[1] = secondary_indices[0]
        elif len(candidates) >= 2:
            best: Optional[tuple[float, int, int]] = None
            for first in range(len(candidates)):
                for second in range(len(candidates)):
                    if first == second:
                        continue
                    first_jump = float(np.linalg.norm(
                        centroids[first] - self.bundle_track_centroids[0]
                    ))
                    second_jump = float(np.linalg.norm(
                        centroids[second] - self.bundle_track_centroids[1]
                    ))
                    second_max_jump = (
                        self.b2_left_track_max_jump
                        if self.reverse_pair_confirmed else self.bundle_track_max_jump
                    )
                    if (first_jump > self.bundle_track_max_jump
                            or second_jump > second_max_jump):
                        continue
                    candidate_cost = first_jump + second_jump
                    if best is None or candidate_cost < best[0]:
                        best = (candidate_cost, first, second)
            if best is not None:
                assignments = {0: best[1], 1: best[2]}

        # If only one vehicle is visible after both IDs were acquired, keep
        # the nearest identity but do not claim that both are visible.
        if not assignments and self.bundle_track_centroids[0] is not None:
            best_single: Optional[tuple[float, int, int]] = None
            for track_index in (0, 1):
                if self.bundle_track_centroids[track_index] is None:
                    continue
                for candidate_index, centroid in enumerate(centroids):
                    jump = float(np.linalg.norm(
                        centroid - self.bundle_track_centroids[track_index]
                    ))
                    max_jump = (
                        self.b2_left_track_max_jump
                        if track_index == 1 and self.reverse_pair_confirmed
                        else self.bundle_track_max_jump
                    )
                    if jump <= max_jump:
                        if best_single is None or jump < best_single[0]:
                            best_single = (jump, track_index, candidate_index)
            if best_single is not None:
                assignments = {best_single[1]: best_single[2]}

        for track_index, candidate_index in assignments.items():
            bundle = candidates[candidate_index]
            self.bundle_track_centroids[track_index] = centroids[candidate_index]
            self.bundle_track_distances[track_index] = self.bundle_distance(bundle)
            if track_index == 0:
                self.observation.bundle1 = bundle
                self.observation.bundle1_visible = True
                self.observation.bundle1_distance = self.bundle_track_distances[0]
            else:
                self.observation.bundle2 = bundle
                self.observation.bundle2_visible = True
                self.observation.bundle2_distance = self.bundle_track_distances[1]

    def transition(self, next_state: ParkingState, now: float) -> None:
        if self.state != next_state:
            self.get_logger().info(f'Parking: {self.state.value} -> {next_state.value}')
            self.state = next_state
            self.state_started_at = now
            if next_state == ParkingState.WAIT_RIGHT_CLEAR_INITIAL:
                self.approach_started_at = now
            elif next_state == ParkingState.PASS_SECOND_CAR:
                self.capture_reverse_primary_seed()
            elif next_state == ParkingState.REVERSE_HARD_RIGHT:
                self.reset_bundle_tracking()

    def control_tick(self) -> None:
        now = time.monotonic()
        if self.last_scan_at is None:
            self.publish(0, 0)
            return
        if now - self.last_scan_at > self.scan_timeout_sec:
            self.transition(ParkingState.EMERGENCY_STOP, now)
            self.publish(0, 0)
            return
        if self.invalid_scan_count > 0:
            if self.invalid_scan_count >= self.invalid_scan_confirm_frames:
                self.get_logger().error('Emergency stop: invalid LiDAR scan stream')
                self.transition(ParkingState.EMERGENCY_STOP, now)
            self.publish(0, 0)
            return
        if self.state == ParkingState.WAIT_FOR_SCAN:
            self.transition(ParkingState.WAIT_RIGHT_CLEAR_INITIAL, now)

        elapsed = now - self.state_started_at
        right_visible = self.observation.right_visible

        if (self.state == ParkingState.REVERSE_HARD_RIGHT
                and self.observation.rear_axis_min_distance is not None
                and self.observation.rear_axis_min_distance <= self.rear_hard_stop_distance):
            self.get_logger().error('Emergency stop: rear obstacle too close')
            self.transition(ParkingState.PARKING_FAILED, now)
            self.publish(0, 0)
            return

        if (self.state in (
                ParkingState.WAIT_RIGHT_CLEAR_INITIAL,
                ParkingState.WAIT_CAR1_ENTRY,
                ParkingState.WAIT_CAR2_ENTRY,
        ) and self.approach_started_at is not None
                and now - self.approach_started_at >= self.approach_timeout_sec):
            self.get_logger().error('Parking failed: approach detection timeout')
            self.transition(ParkingState.PARKING_FAILED, now)
            self.publish(0, 0)
            return

        if self.state == ParkingState.WAIT_RIGHT_CLEAR_INITIAL:
            # Ignore every RIGHT-side object visible at start. Only after the
            # RIGHT valid side is empty do we begin counting parked cars.
            if not right_visible and self.right_empty_since is None:
                self.right_empty_since = now
            elif right_visible:
                self.right_empty_since = None
            if (self.right_empty_since is not None
                    and now - self.right_empty_since >= self.right_empty_confirm_sec):
                self.right_entry_frames = 0
                self.right_empty_since = None
                self.transition(ParkingState.WAIT_CAR1_ENTRY, now)
            self.publish(0, self.forward_speed)
            return

        if self.state == ParkingState.WAIT_CAR1_ENTRY:
            if self.right_entry_frames >= self.right_entry_confirm_frames:
                self.get_logger().info('RIGHT single bundle #1 detected; waiting for car #2')
                self.right_entry_frames = 0
                self.right_two_bundles_seen = False
                self.transition(ParkingState.WAIT_CAR2_ENTRY, now)
            self.publish(0, self.forward_speed)
            return

        if self.state == ParkingState.WAIT_CAR2_ENTRY:
            if self.right_entry_frames >= self.right_entry_confirm_frames:
                self.right_entry_frames = 0
                if not self.right_two_bundles_seen:
                    self.right_two_bundles_seen = True
                    self.get_logger().info('RIGHT two bundles detected; waiting until only one remains')
                else:
                    self.get_logger().info('RIGHT returned to one bundle; starting parking entry')
                    self.transition(ParkingState.PASS_SECOND_CAR, now)
            self.publish(0, self.forward_speed)
            return

        if self.state == ParkingState.PASS_SECOND_CAR:
            # After car 2 passes, move forward while gently turning left before
            # locking maximum right steering for the hard-coded reverse.
            if elapsed >= self.pass_second_car_sec:
                self.transition(ParkingState.SET_REVERSE_STEER, now)
            self.publish(self.pre_reverse_left_steer, self.forward_speed)
            return

        if self.state == ParkingState.SET_REVERSE_STEER:
            if elapsed >= self.steer_settle_sec:
                self.transition(ParkingState.REVERSE_HARD_RIGHT, now)
            self.publish(self.right_turn_steer, 0)
            return

        if self.state == ParkingState.REVERSE_HARD_RIGHT:
            # Keep hard-coded reverse until B2 approached the vehicle and
            # then starts moving away again (its center-distance turnaround).
            if (self.reverse_pair_confirmed
                    and self.reverse_b2_turnaround_detected):
                self.transition(ParkingState.PARK_STOP, now)
                self.publish(0, 0)
                return
            elif elapsed >= self.reverse_seek_timeout_sec:
                self.get_logger().error(
                    'Parking failed: B1/B2 acquisition or centering timed out'
                )
                self.transition(ParkingState.PARKING_FAILED, now)
                self.publish(0, 0)
                return
            self.publish(self.right_turn_steer, self.reverse_speed)
            return

        if self.state == ParkingState.PARK_STOP:
            if elapsed >= self.park_stop_sec:
                self.transition(ParkingState.EXIT_SET_RIGHT_STEER, now)
            self.publish(0, 0)
            return

        if self.state == ParkingState.EXIT_SET_RIGHT_STEER:
            if elapsed >= self.steer_settle_sec:
                self.transition(ParkingState.EXIT_RIGHT_TURN, now)
                self.publish(self.right_turn_steer, self.forward_speed)
            else:
                self.publish(self.right_turn_steer, 0)
            return

        if self.state == ParkingState.EXIT_RIGHT_TURN:
            if elapsed >= self.exit_right_turn_sec:
                self.transition(ParkingState.EXIT_FORWARD, now)
                self.publish(0, self.forward_speed)
            else:
                self.publish(self.right_turn_steer, self.forward_speed)
            return

        if self.state == ParkingState.EXIT_FORWARD:
            if elapsed >= self.exit_forward_sec:
                self.transition(ParkingState.DONE, now)
            self.publish(0, self.forward_speed)
            return

        if self.state == ParkingState.DONE:
            # Parking mission is complete; continue along the original
            # straight travel direction until a higher-level node takes over.
            self.publish(0, self.forward_speed)
            return

        self.publish(0, 0)  # PARKING_FAILED or EMERGENCY_STOP

    def publish(self, steer: int, speed: int) -> None:
        self.last_command = (int(np.clip(steer, -45, 45)), int(speed))
        message = Int16MultiArray()
        message.data = list(self.last_command)
        self.motor_pub.publish(message)

    def destroy_node(self):
        """Always send a final stop command on Ctrl+C or launch shutdown."""
        try:
            self.publish(0, 0)
        except Exception:
            # ROS publisher/context may already be torn down during shutdown.
            pass
        if self.debug_view:
            cv2.destroyAllWindows()
        return super().destroy_node()

    def draw_debug(self) -> None:
        """Visualise the active parking sequence and valid-side car bundles."""
        size = 760
        center = size // 2
        scale = (size * 0.40) / max(self.cluster_max_range, 0.1)
        image = np.zeros((size, size, 3), dtype=np.uint8)

        # Parking-valid field is rear-centered and split LEFT/RIGHT.
        radius = int(self.cluster_max_range * scale)
        cv2.circle(image, (center, center), radius, (0, 120, 120), 1)
        for boundary_angle in (-self.valid_sector_max_abs, self.valid_sector_max_abs):
            endpoint_x = int(center + math.sin(boundary_angle) * radius)
            endpoint_y = int(center + math.cos(boundary_angle) * radius)
            cv2.line(image, (center, center), (endpoint_x, endpoint_y),
                     (0, 120, 120), 1)
        cv2.circle(image, (center, center), 8, (0, 255, 0), -1)
        cv2.putText(image, 'CAR', (center + 12, center + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        valid_angle_deg = math.degrees(self.valid_sector_max_abs)
        cv2.putText(image, f'VALID: |ANGLE| <= {valid_angle_deg:.0f} DEG (LEFT / RIGHT)',
                    (center - 185, center + radius - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 220), 1, cv2.LINE_AA)

        for bundle in self.observation.vehicle_bundles:
            for x_forward, y_left in bundle:
                x = int(center - y_left * scale)
                y = int(center - x_forward * scale)
                cv2.circle(image, (x, y), 1, (90, 90, 90), -1)

        tracked_bundles = [self.observation.bundle1, self.observation.bundle2]
        for bundle_index, bundle in enumerate(tracked_bundles):
            if bundle is None:
                continue
            color = (0, 255, 0) if bundle_index == 0 else (0, 180, 255)
            for x_forward, y_left in bundle:
                x = int(center - y_left * scale)
                y = int(center - x_forward * scale)
                cv2.circle(image, (x, y), 2, color, -1)
            centroid = np.median(bundle, axis=0)
            label_x = int(center - centroid[1] * scale)
            label_y = int(center - centroid[0] * scale)
            distance = self.bundle_distance(bundle)
            cv2.putText(image, f'B{bundle_index + 1}: {distance:.2f}m',
                        (label_x + 6, label_y - 6), cv2.FONT_HERSHEY_SIMPLEX,
                        0.48, color, 1, cv2.LINE_AA)

        steps = [
            ParkingState.WAIT_FOR_SCAN,
            ParkingState.WAIT_RIGHT_CLEAR_INITIAL, ParkingState.WAIT_CAR1_ENTRY,
            ParkingState.WAIT_CAR2_ENTRY,
            ParkingState.PASS_SECOND_CAR,
            ParkingState.SET_REVERSE_STEER, ParkingState.REVERSE_HARD_RIGHT,
            ParkingState.PARK_STOP,
            ParkingState.EXIT_SET_RIGHT_STEER,
            ParkingState.EXIT_RIGHT_TURN, ParkingState.EXIT_FORWARD,
            ParkingState.DONE, ParkingState.PARKING_FAILED,
        ]
        cv2.rectangle(image, (8, 8), (size - 8, 260), (50, 50, 50), 1)
        cv2.putText(image, f'STATE: {self.state.value}', (18, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2, cv2.LINE_AA)
        bundle1 = '-' if self.observation.bundle1_distance is None else f'{self.observation.bundle1_distance:.2f}'
        bundle2 = '-' if self.observation.bundle2_distance is None else f'{self.observation.bundle2_distance:.2f}'
        right_stage = 'pair-seen' if self.right_two_bundles_seen else 'waiting-pair'
        cv2.putText(image, f'cars={self.observation.vehicle_bundle_count} right-cars={self.observation.right_bundle_count} raw L/R={self.observation.left_point_count}/{self.observation.right_point_count} ({right_stage}) B1={bundle1} B2={bundle2}',
                    (18, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
        pair_state = 'ready' if self.reverse_pair_confirmed else (
            f'{self.reverse_pair_confirm_count}/{self.reverse_pair_confirm_frames}'
        )
        b2_turn = 'detected' if self.reverse_b2_turnaround_detected else (
            'approaching' if self.reverse_b2_has_approached else 'waiting'
        )
        cv2.putText(image, f'cmd: steer={self.last_command[0]}, speed={self.last_command[1]} pair={pair_state} B2-turn={b2_turn}',
                    (18, 112), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
        for index, step in enumerate(steps):
            col, row = index % 2, index // 2
            color = (0, 255, 0) if step == self.state else (165, 165, 165)
            prefix = '>> ' if step == self.state else '   '
            cv2.putText(image, prefix + step.value, (18 + col * 365, 138 + row * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)

        cv2.imshow(self.debug_window_name, image)
        cv2.waitKey(1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingNodeOsy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0, 0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
