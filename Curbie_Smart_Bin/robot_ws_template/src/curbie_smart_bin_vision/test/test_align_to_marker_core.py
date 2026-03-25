"""Tests for target-aware ArUco alignment logic."""

import math

import pytest

from curbie_smart_bin_vision.align_to_marker_core import AlignMeasurement
from curbie_smart_bin_vision.align_to_marker_core import AlignState
from curbie_smart_bin_vision.align_to_marker_core import AlignToMarkerCore
from curbie_smart_bin_vision.align_to_marker_core import select_target_measurement


def test_select_target_measurement_ignores_other_marker_ids():
    measurements = {
        1: AlignMeasurement(1, 0.0, 0.5, 0.0, 0.0),
        9: AlignMeasurement(9, 0.2, 0.7, 0.1, 0.0),
    }

    selected = select_target_measurement(measurements, 9)

    assert selected is not None
    assert selected.marker_id == 9


def test_align_core_succeeds_after_hold_at_requested_slot_offset():
    core = AlignToMarkerCore(
        stand_off_m=0.45,
        lateral_offset_m=0.18,
        yaw_offset_rad=0.0,
        timeout_s=10.0,
        success_hold_steps=3,
    )
    core.start(now_s=0.0)

    measurement = AlignMeasurement(
        marker_id=9,
        x_m=0.18,
        z_m=0.45,
        marker_yaw_rad=0.0,
        stamp_s=0.1,
    )

    core.step(now_s=0.1, measurement=measurement)
    core.step(now_s=0.2, measurement=measurement)
    command = core.step(now_s=0.3, measurement=measurement)

    assert core.state == AlignState.SUCCEEDED
    assert command.linear_x == pytest.approx(0.0)
    assert command.angular_z == pytest.approx(0.0)


def test_align_core_retries_lost_marker_then_fails():
    core = AlignToMarkerCore(
        stand_off_m=0.30,
        lateral_offset_m=0.0,
        yaw_offset_rad=0.0,
        timeout_s=20.0,
        search_spin_speed_rad_s=math.tau,
        lost_marker_timeout_s=0.1,
        lost_scan_timeout_s=0.2,
        max_lost_retries=1,
    )
    core.start(now_s=0.0)

    seen = AlignMeasurement(7, 0.0, 0.6, 0.0, 0.05)
    core.step(now_s=0.05, measurement=seen)
    assert core.state in {AlignState.COARSE_ALIGN, AlignState.FINE_ALIGN, AlignState.HOLDING}

    core.step(now_s=0.3, measurement=None)
    assert core.state == AlignState.LOST_SCAN
    core.step(now_s=2.4, measurement=None)
    core.step(now_s=4.5, measurement=None)

    assert core.state == AlignState.FAILED


def test_align_core_alternates_direction_after_each_full_search_sweep():
    core = AlignToMarkerCore(
        stand_off_m=0.30,
        lateral_offset_m=0.0,
        yaw_offset_rad=0.0,
        timeout_s=20.0,
        search_spin_speed_rad_s=math.tau,
        lost_marker_timeout_s=0.1,
        max_lost_retries=10,
    )
    core.start(now_s=0.0)

    first = core.step(now_s=0.2, measurement=None)
    second = core.step(now_s=1.3, measurement=None)
    third = core.step(now_s=2.4, measurement=None)
    fourth = core.step(now_s=4.5, measurement=None)

    assert first.angular_z == pytest.approx(math.tau)
    assert second.angular_z == pytest.approx(math.tau)
    assert third.angular_z == pytest.approx(-math.tau)
    assert fourth.angular_z == pytest.approx(math.tau)
