# Copyright 2024 Fola
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Core state machine for ArUco marker search and approach behavior."""

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class SeekState(str, Enum):
    """Autonomous behavior states."""

    IDLE = 'IDLE'
    SEARCHING = 'SEARCHING'
    APPROACHING = 'APPROACHING'
    RETRY_NUDGE = 'RETRY_NUDGE'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'


@dataclass(frozen=True)
class VelocityCommand:
    """Velocity command in robot base frame."""

    linear_x: float
    angular_z: float


@dataclass(frozen=True)
class MarkerMeasurement:
    """Latest marker measurement in camera frame."""

    x_m: float
    z_m: float
    stamp_s: float


class MarkerSeekCore:
    """Pure state machine for marker search and approach behavior."""

    def __init__(
        self,
        stop_distance_m: float = 0.25,
        spin_speed_rad_s: float = 0.60,
        search_timeout_s: float = 12.0,
        max_retries: int = 3,
        retry_nudge_speed_m_s: float = 0.10,
        retry_nudge_duration_s: float = 1.5,
        approach_speed_cap_m_s: float = 0.10,
        yaw_kp: float = 1.5,
        distance_kp: float = 0.8,
        lost_marker_timeout_s: float = 0.6,
        heading_align_threshold_rad: float = 0.05,
        heading_stop_yaw_rad: float = 0.15,
    ) -> None:
        """Initialize configurable search/approach behavior parameters."""
        self.stop_distance_m = stop_distance_m
        self.spin_speed_rad_s = spin_speed_rad_s
        self.search_timeout_s = search_timeout_s
        self.max_retries = max_retries
        self.retry_nudge_speed_m_s = retry_nudge_speed_m_s
        self.retry_nudge_duration_s = retry_nudge_duration_s
        self.approach_speed_cap_m_s = approach_speed_cap_m_s
        self.yaw_kp = yaw_kp
        self.distance_kp = distance_kp
        self.lost_marker_timeout_s = lost_marker_timeout_s
        self.heading_align_threshold_rad = heading_align_threshold_rad
        self.heading_stop_yaw_rad = heading_stop_yaw_rad
        if self.heading_stop_yaw_rad <= self.heading_align_threshold_rad:
            self.heading_stop_yaw_rad = self.heading_align_threshold_rad + 0.05

        self.state = SeekState.IDLE
        self.state_start_time_s = 0.0
        self.retries_used = 0
        self.latest_marker: Optional[MarkerMeasurement] = None

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min(value, max_value), min_value)

    def _transition(self, next_state: SeekState, now_s: float) -> None:
        self.state = next_state
        self.state_start_time_s = now_s

    def start(self, now_s: float) -> Tuple[bool, str]:
        """Start autonomous behavior if current state allows it."""
        allowed = {SeekState.IDLE, SeekState.FAILED, SeekState.SUCCEEDED}
        if self.state not in allowed:
            return False, f'Cannot start while in {self.state.value}'

        self.retries_used = 0
        self.latest_marker = None
        self._transition(SeekState.SEARCHING, now_s)
        return True, 'Marker seek started'

    def stop(self, now_s: float) -> Tuple[bool, str]:
        """Stop autonomous behavior and return to IDLE state."""
        self.latest_marker = None
        self._transition(SeekState.IDLE, now_s)
        return True, 'Marker seek stopped'

    def set_marker_measurement(
        self,
        x_m: float,
        z_m: float,
        now_s: float,
    ) -> None:
        """Update latest marker and trigger approach when currently searching."""
        if z_m <= 0.0:
            return

        self.latest_marker = MarkerMeasurement(x_m=x_m, z_m=z_m, stamp_s=now_s)
        if self.state == SeekState.SEARCHING:
            self._transition(SeekState.APPROACHING, now_s)

    def _marker_fresh(self, now_s: float) -> bool:
        if self.latest_marker is None:
            return False
        age = now_s - self.latest_marker.stamp_s
        return age <= self.lost_marker_timeout_s

    def _search_command(self) -> VelocityCommand:
        return VelocityCommand(linear_x=0.0, angular_z=self.spin_speed_rad_s)

    def _stop_command(self) -> VelocityCommand:
        return VelocityCommand(linear_x=0.0, angular_z=0.0)

    def _step_searching(self, now_s: float) -> VelocityCommand:
        elapsed = now_s - self.state_start_time_s
        if elapsed < self.search_timeout_s:
            return self._search_command()

        if self.retries_used < self.max_retries:
            self._transition(SeekState.RETRY_NUDGE, now_s)
            return VelocityCommand(
                linear_x=self.retry_nudge_speed_m_s,
                angular_z=0.0,
            )

        self._transition(SeekState.FAILED, now_s)
        return self._stop_command()

    def _step_approaching(self, now_s: float) -> VelocityCommand:
        if not self._marker_fresh(now_s):
            self._transition(SeekState.SEARCHING, now_s)
            return self._search_command()

        assert self.latest_marker is not None
        x_m = self.latest_marker.x_m
        z_m = self.latest_marker.z_m

        if z_m <= self.stop_distance_m:
            self._transition(SeekState.SUCCEEDED, now_s)
            return self._stop_command()

        yaw_error = math.atan2(x_m, z_m)
        angular = self._clamp(
            self.yaw_kp * yaw_error,
            -self.spin_speed_rad_s,
            self.spin_speed_rad_s,
        )

        linear_error = z_m - self.stop_distance_m
        linear_base = self._clamp(
            self.distance_kp * linear_error,
            0.0,
            self.approach_speed_cap_m_s,
        )

        # Prefer rotate-first alignment so approach is less curved when heading error is large.
        yaw_abs = abs(yaw_error)
        if yaw_abs >= self.heading_stop_yaw_rad:
            linear_scale = 0.0
        elif yaw_abs <= self.heading_align_threshold_rad:
            linear_scale = 1.0
        else:
            span = self.heading_stop_yaw_rad - self.heading_align_threshold_rad
            linear_scale = (self.heading_stop_yaw_rad - yaw_abs) / max(1e-6, span)

        linear = linear_base * linear_scale
        return VelocityCommand(linear_x=linear, angular_z=angular)

    def _step_retry_nudge(self, now_s: float) -> VelocityCommand:
        elapsed = now_s - self.state_start_time_s
        if elapsed < self.retry_nudge_duration_s:
            return VelocityCommand(
                linear_x=self.retry_nudge_speed_m_s,
                angular_z=0.0,
            )

        self.retries_used += 1
        self.latest_marker = None
        self._transition(SeekState.SEARCHING, now_s)
        return self._search_command()

    def step(self, now_s: float) -> VelocityCommand:
        """Advance state machine by one control step."""
        if self.state == SeekState.IDLE:
            return self._stop_command()
        if self.state == SeekState.SEARCHING:
            return self._step_searching(now_s)
        if self.state == SeekState.APPROACHING:
            return self._step_approaching(now_s)
        if self.state == SeekState.RETRY_NUDGE:
            return self._step_retry_nudge(now_s)
        if self.state == SeekState.SUCCEEDED:
            return self._stop_command()
        if self.state == SeekState.FAILED:
            return self._stop_command()
        return self._stop_command()
