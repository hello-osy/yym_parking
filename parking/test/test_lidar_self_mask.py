import math
from pathlib import Path

import numpy as np
import yaml

from parking.lidar_safety import LidarSafetySelfMask, SelfMaskRegion
from parking.models import SafetyDecision
from parking.parking_node import ParkingNode


LEFT_SIDE_SECTOR = (-0.75, 0.60, 0.10, 1.25)
SELF_RETURN = (0.085, 0.145)
HARD_STOP_DISTANCE = 0.20


def _mask(enabled=True):
    return LidarSafetySelfMask(
        enabled,
        (
            SelfMaskRegion(
                "lidar_front_left_self_return",
                0.04,
                0.13,
                0.11,
                0.18,
            ),
        ),
    )


def _node_shell(mask_enabled=True):
    node = ParkingNode.__new__(ParkingNode)
    node.left_side_sector = LEFT_SIDE_SECTOR
    node.left_side_hard_stop_distance = HARD_STOP_DISTANCE
    node.lidar_self_mask = _mask(mask_enabled)
    node.left_side_lidar_minimum_before_mask = math.inf
    node.left_side_lidar_minimum_after_mask = math.inf
    node.left_side_lidar_minimum_point_after_mask = None
    node.lidar_self_mask_filtered_count = 0
    node.lidar_self_mask_matched_regions = ()
    return node


def _clear_ultrasonic_safety():
    return SafetyDecision(False, 1.0, 0.0, "clear", math.inf)


def test_masked_self_return_alone_does_not_trigger_left_side_hard_stop():
    node = _node_shell()
    node._update_left_side_lidar_safety(np.asarray([SELF_RETURN]))

    assert node.left_side_lidar_minimum_before_mask < HARD_STOP_DISTANCE
    assert math.isinf(node.left_side_lidar_minimum_after_mask)
    assert node.lidar_self_mask_filtered_count == 1
    assert node.lidar_self_mask_matched_regions == (
        "lidar_front_left_self_return",
    )
    assert not node._apply_left_side_lidar_hard_stop(
        _clear_ultrasonic_safety()
    ).hard_stop


def test_external_point_outside_mask_remains_a_left_side_hard_stop():
    external_obstacle = (-0.12, 0.14)
    node = _node_shell()
    node._update_left_side_lidar_safety(
        np.asarray([SELF_RETURN, external_obstacle])
    )
    decision = node._apply_left_side_lidar_hard_stop(
        _clear_ultrasonic_safety()
    )

    assert node.lidar_self_mask_filtered_count == 1
    assert node.left_side_lidar_minimum_point_after_mask == external_obstacle
    assert node.left_side_lidar_minimum_after_mask < HARD_STOP_DISTANCE
    assert decision.hard_stop
    assert decision.reason == "LIDAR_HARD_STOP:left_side"


def test_disabling_mask_restores_existing_self_return_hard_stop_behavior():
    node = _node_shell(mask_enabled=False)
    node._update_left_side_lidar_safety(np.asarray([SELF_RETURN]))
    decision = node._apply_left_side_lidar_hard_stop(
        _clear_ultrasonic_safety()
    )

    assert node.lidar_self_mask_filtered_count == 0
    assert node.left_side_lidar_minimum_before_mask == (
        node.left_side_lidar_minimum_after_mask
    )
    assert node.left_side_lidar_minimum_after_mask < HARD_STOP_DISTANCE
    assert decision.hard_stop


def test_safety_mask_does_not_mutate_slot_detector_input():
    points = np.asarray(
        [SELF_RETURN, (-0.12, 0.14), (0.80, 0.90)], dtype=np.float64
    )
    detector_input = points.copy()

    _mask().minimum_in_sector(points, LEFT_SIDE_SECTOR)

    np.testing.assert_array_equal(points, detector_input)


def test_speed_steering_and_hard_stop_configuration_remain_fixed():
    config_path = Path(__file__).parents[1] / "config" / "parking.yaml"
    params = yaml.safe_load(config_path.read_text())["parking_node"][
        "ros__parameters"
    ]

    assert params["control.approach_speed"] == 22
    assert params["control.reverse_arc_speed"] == -18
    assert params["control.align_speed"] == -14
    assert params["control.final_reverse_speed"] == -12
    assert params["control.exit_speed"] == 20
    assert params["control.left_slot_steer"] == 45
    assert params["control.right_slot_steer"] == -45
    assert params["lidar.left_side_hard_stop_distance"] == 0.20
    assert params["lidar_safety.self_mask.enabled"] is True
