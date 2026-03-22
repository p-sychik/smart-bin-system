"""Pure target-aware alignment logic for mission docking."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional

from turtlebot3_marker_seek_demo2.marker_seek_core import VelocityCommand


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


class AlignState(str, Enum):
    """Alignment controller states."""

    SEARCHING = 'SEARCHING'
    COARSE_ALIGN = 'COARSE_ALIGN'
    FINE_ALIGN = 'FINE_ALIGN'
    HOLDING = 'HOLDING'
    LOST_SCAN = 'LOST_SCAN'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'


@dataclass(frozen=True)
class AlignMeasurement:
    """Target marker observation in the camera frame."""

    marker_id: int
    x_m: float
    z_m: float
    marker_yaw_rad: float
    stamp_s: float


def select_target_measurement(
    measurements: Dict[int, AlignMeasurement],
    target_marker_id: int,
) -> Optional[AlignMeasurement]:
    """Return the desired marker observation when present."""
    return measurements.get(int(target_marker_id))


class AlignToMarkerCore:
    """Target-aware staged controller for final ArUco alignment."""

    def __init__(
        self,
        stand_off_m: float,
        lateral_offset_m: float,
        yaw_offset_rad: float,
        timeout_s: float,
        search_spin_speed_rad_s: float = 0.45,
        distance_kp: float = 0.8,
        lateral_kp: float = 1.4,
        yaw_kp: float = 0.8,
        max_linear_speed_m_s: float = 0.10,
        max_reverse_speed_m_s: float = 0.05,
        max_angular_speed_rad_s: float = 0.8,
        coarse_align_threshold_rad: float = 0.25,
        linear_gate_yaw_rad: float = 0.35,
        distance_tolerance_m: float = 0.04,
        lateral_tolerance_m: float = 0.04,
        yaw_tolerance_rad: float = 0.10,
        success_hold_steps: int = 5,
        lost_marker_timeout_s: float = 0.6,
        lost_scan_timeout_s: float = 1.5,
        max_lost_retries: int = 3,
    ) -> None:
        self.stand_off_m = max(0.05, stand_off_m)
        self.lateral_offset_m = lateral_offset_m
        self.yaw_offset_rad = yaw_offset_rad
        self.timeout_s = max(1.0, timeout_s)
        self.search_spin_speed_rad_s = max(0.1, search_spin_speed_rad_s)
        self.distance_kp = max(0.1, distance_kp)
        self.lateral_kp = max(0.1, lateral_kp)
        self.yaw_kp = max(0.1, yaw_kp)
        self.max_linear_speed_m_s = max(0.02, max_linear_speed_m_s)
        self.max_reverse_speed_m_s = max(0.0, max_reverse_speed_m_s)
        self.max_angular_speed_rad_s = max(0.2, max_angular_speed_rad_s)
        self.coarse_align_threshold_rad = max(0.05, coarse_align_threshold_rad)
        self.linear_gate_yaw_rad = max(
            self.coarse_align_threshold_rad,
            linear_gate_yaw_rad,
        )
        self.distance_tolerance_m = max(0.01, distance_tolerance_m)
        self.lateral_tolerance_m = max(0.01, lateral_tolerance_m)
        self.yaw_tolerance_rad = max(0.02, yaw_tolerance_rad)
        self.success_hold_steps = max(1, success_hold_steps)
        self.lost_marker_timeout_s = max(0.05, lost_marker_timeout_s)
        self.lost_scan_timeout_s = max(0.2, lost_scan_timeout_s)
        self.max_lost_retries = max(0, max_lost_retries)

        self.state = AlignState.SEARCHING
        self.start_time_s = 0.0
        self._hold_steps = 0
        self._lost_scan_started_s: Optional[float] = None
        self._lost_retries = 0
        self._last_turn_sign = 1.0

        self.last_lateral_error_m = 0.0
        self.last_distance_error_m = 0.0
        self.last_yaw_error_rad = 0.0

    def start(self, now_s: float) -> None:
        self.state = AlignState.SEARCHING
        self.start_time_s = now_s
        self._hold_steps = 0
        self._lost_scan_started_s = None
        self._lost_retries = 0
        self._last_turn_sign = 1.0
        self.last_lateral_error_m = 0.0
        self.last_distance_error_m = 0.0
        self.last_yaw_error_rad = 0.0

    def _timed_out(self, now_s: float) -> bool:
        return now_s - self.start_time_s > self.timeout_s

    def _within_tolerance(self) -> bool:
        return (
            abs(self.last_distance_error_m) <= self.distance_tolerance_m
            and abs(self.last_lateral_error_m) <= self.lateral_tolerance_m
            and abs(self.last_yaw_error_rad) <= self.yaw_tolerance_rad
        )

    def _search_command(self) -> VelocityCommand:
        return VelocityCommand(linear_x=0.0, angular_z=self._last_turn_sign * self.search_spin_speed_rad_s)

    def _stop_command(self) -> VelocityCommand:
        return VelocityCommand(linear_x=0.0, angular_z=0.0)

    def _update_errors(self, measurement: AlignMeasurement) -> None:
        self.last_lateral_error_m = measurement.x_m - self.lateral_offset_m
        self.last_distance_error_m = measurement.z_m - self.stand_off_m
        self.last_yaw_error_rad = wrap_angle(measurement.marker_yaw_rad - self.yaw_offset_rad)
        if abs(self.last_lateral_error_m) > 1e-3:
            self._last_turn_sign = 1.0 if self.last_lateral_error_m > 0.0 else -1.0

    def step(
        self,
        now_s: float,
        measurement: Optional[AlignMeasurement],
    ) -> VelocityCommand:
        """Advance the alignment controller by one step."""
        if self._timed_out(now_s):
            self.state = AlignState.FAILED
            return self._stop_command()

        measurement_fresh = (
            measurement is not None
            and now_s - measurement.stamp_s <= self.lost_marker_timeout_s
        )
        if not measurement_fresh:
            self._hold_steps = 0
            if self._lost_scan_started_s is None:
                self._lost_scan_started_s = now_s
                self.state = AlignState.LOST_SCAN
                return self._search_command()

            if now_s - self._lost_scan_started_s > self.lost_scan_timeout_s:
                self._lost_retries += 1
                self._lost_scan_started_s = now_s
                if self._lost_retries > self.max_lost_retries:
                    self.state = AlignState.FAILED
                    return self._stop_command()
            self.state = AlignState.LOST_SCAN
            return self._search_command()

        assert measurement is not None
        self._lost_scan_started_s = None
        self._update_errors(measurement)

        if self._within_tolerance():
            self._hold_steps += 1
            self.state = AlignState.HOLDING
            if self._hold_steps >= self.success_hold_steps:
                self.state = AlignState.SUCCEEDED
                return self._stop_command()
            return self._stop_command()

        self._hold_steps = 0
        lateral_heading = math.atan2(
            self.last_lateral_error_m,
            max(0.05, measurement.z_m),
        )
        combined_heading = wrap_angle(
            lateral_heading + 0.6 * self.last_yaw_error_rad
        )
        if (
            abs(combined_heading) > self.coarse_align_threshold_rad
            or abs(self.last_yaw_error_rad) > self.coarse_align_threshold_rad
        ):
            self.state = AlignState.COARSE_ALIGN
        else:
            self.state = AlignState.FINE_ALIGN

        angular = self.lateral_kp * lateral_heading + self.yaw_kp * self.last_yaw_error_rad
        angular = max(
            -self.max_angular_speed_rad_s,
            min(self.max_angular_speed_rad_s, angular),
        )

        distance_command = self.distance_kp * self.last_distance_error_m
        distance_command = max(
            -self.max_reverse_speed_m_s,
            min(self.max_linear_speed_m_s, distance_command),
        )
        if abs(combined_heading) >= self.linear_gate_yaw_rad:
            linear = 0.0
        else:
            linear_scale = max(0.25, 1.0 - abs(combined_heading) / self.linear_gate_yaw_rad)
            linear = distance_command * linear_scale
        return VelocityCommand(linear_x=linear, angular_z=angular)
