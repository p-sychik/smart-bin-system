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
"""Unit tests for marker seek state transitions and control outputs."""

import pytest

from curbie_smart_bin_vision.marker_seek_core import MarkerSeekCore
from curbie_smart_bin_vision.marker_seek_core import SeekState


def test_start_from_idle_enters_searching():
    """Starting from IDLE should enter SEARCHING."""
    core = MarkerSeekCore()
    success, _ = core.start(now_s=0.0)

    assert success
    assert core.state == SeekState.SEARCHING


def test_stop_from_active_state_returns_idle():
    """Stopping from active mode should zero command and return IDLE."""
    core = MarkerSeekCore()
    core.start(now_s=0.0)
    cmd = core.step(now_s=0.1)
    assert cmd.angular_z > 0.0

    success, _ = core.stop(now_s=0.2)
    after_stop = core.step(now_s=0.3)

    assert success
    assert core.state == SeekState.IDLE
    assert after_stop.linear_x == pytest.approx(0.0)
    assert after_stop.angular_z == pytest.approx(0.0)


def test_search_timeout_goes_to_retry_then_back_to_search():
    """Search timeout should trigger retry nudge and return to SEARCHING."""
    core = MarkerSeekCore()
    core.start(now_s=0.0)

    retry_cmd = core.step(now_s=12.01)
    assert core.state == SeekState.RETRY_NUDGE
    assert retry_cmd.linear_x == pytest.approx(0.10)

    search_cmd = core.step(now_s=13.60)
    assert core.state == SeekState.SEARCHING
    assert core.retries_used == 1
    assert search_cmd.angular_z == pytest.approx(0.60)


def test_state_becomes_failed_after_max_retries():
    """Node should fail safely once retry budget is exhausted."""
    core = MarkerSeekCore()
    core.start(now_s=0.0)
    now_s = 0.0

    for expected_retry in range(1, 4):
        now_s += 12.01
        core.step(now_s=now_s)
        assert core.state == SeekState.RETRY_NUDGE

        now_s += 1.60
        core.step(now_s=now_s)
        assert core.state == SeekState.SEARCHING
        assert core.retries_used == expected_retry

    now_s += 12.01
    final_cmd = core.step(now_s=now_s)
    assert core.state == SeekState.FAILED
    assert final_cmd.linear_x == pytest.approx(0.0)
    assert final_cmd.angular_z == pytest.approx(0.0)


def test_detection_transitions_searching_to_approaching():
    """Marker detection should transition SEARCHING to APPROACHING."""
    core = MarkerSeekCore()
    core.start(now_s=0.0)

    core.set_marker_measurement(x_m=0.02, z_m=0.8, now_s=0.5)
    assert core.state == SeekState.APPROACHING


def test_approach_control_clamps_angular_and_gates_forward():
    """Large heading error should clamp turn rate and suppress forward speed."""
    core = MarkerSeekCore(min_linear_scale=0.0)
    core.start(now_s=0.0)
    core.set_marker_measurement(x_m=10.0, z_m=0.5, now_s=0.1)

    cmd = core.step(now_s=0.1)
    assert cmd.linear_x == pytest.approx(0.0)
    assert cmd.angular_z == pytest.approx(0.60)


def test_approach_control_allows_forward_when_aligned():
    """With small heading error, approach should still drive forward."""
    core = MarkerSeekCore()
    core.start(now_s=0.0)
    core.set_marker_measurement(x_m=0.0, z_m=0.5, now_s=0.1)

    cmd = core.step(now_s=0.1)
    assert cmd.linear_x == pytest.approx(0.10)
    assert cmd.angular_z == pytest.approx(0.0)


def test_close_marker_transitions_to_succeeded():
    """Approach should stop and succeed once within stop distance."""
    core = MarkerSeekCore()
    core.start(now_s=0.0)
    core.set_marker_measurement(x_m=0.0, z_m=0.24, now_s=0.1)

    cmd = core.step(now_s=0.1)
    assert core.state == SeekState.SUCCEEDED
    assert cmd.linear_x == pytest.approx(0.0)
    assert cmd.angular_z == pytest.approx(0.0)


def test_stop_distance_tolerance_allows_near_threshold_success():
    """Configured tolerance should allow success slightly above stop distance."""
    core = MarkerSeekCore(
        stop_distance_m=0.25,
        stop_distance_tolerance_m=0.03,
    )
    core.start(now_s=0.0)
    core.set_marker_measurement(x_m=0.0, z_m=0.27, now_s=0.1)

    cmd = core.step(now_s=0.1)
    assert core.state == SeekState.SUCCEEDED
    assert cmd.linear_x == pytest.approx(0.0)
    assert cmd.angular_z == pytest.approx(0.0)


def test_lost_marker_in_approach_returns_to_search():
    """Losing marker during approach should return to SEARCHING."""
    core = MarkerSeekCore()
    core.start(now_s=0.0)
    core.set_marker_measurement(x_m=0.0, z_m=0.7, now_s=0.2)
    core.step(now_s=0.2)
    assert core.state == SeekState.APPROACHING

    cmd = core.step(now_s=1.0)
    assert core.state == SeekState.SEARCHING
    assert cmd.linear_x == pytest.approx(0.0)
    assert cmd.angular_z == pytest.approx(0.60)
