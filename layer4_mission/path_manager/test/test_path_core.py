"""Unit tests for path recording and reverse execution helpers."""

import math

import pytest

from path_manager.path_core import PathFollower
from path_manager.path_core import PathPointRecord
from path_manager.path_core import PathRecorder
from path_manager.path_core import Pose2D
from path_manager.path_core import build_execution_points


def test_path_recorder_downsamples_small_moves():
    recorder = PathRecorder(distance_threshold_m=0.05, yaw_threshold_rad=math.radians(5.0))
    recorder.start(start_timestamp_s=0.0)

    assert recorder.add_pose(Pose2D(0.02, 0.0, 0.0), 0.1) is False
    assert recorder.add_pose(Pose2D(0.06, 0.0, 0.0), 0.2) is True
    recorder.finalize(Pose2D(0.10, 0.0, 0.0), 0.3)

    assert len(recorder.points) == 3
    assert recorder.points[0].x == pytest.approx(0.0)
    assert recorder.points[-1].x == pytest.approx(0.10)


def test_build_execution_points_keeps_forward_path_relative():
    points = [
        PathPointRecord(0.0, 0.0, 0.0, 0.0),
        PathPointRecord(0.5, 0.0, 0.0, 1.0),
        PathPointRecord(1.0, 0.0, 0.0, 2.0),
    ]

    execution = build_execution_points(points, reverse=False)

    assert execution[-1].x == pytest.approx(1.0)
    assert execution[-1].theta == pytest.approx(0.0)


def test_build_execution_points_reverses_geometry_for_forward_drive():
    points = [
        PathPointRecord(0.0, 0.0, 0.0, 0.0),
        PathPointRecord(0.5, 0.0, 0.0, 1.0),
        PathPointRecord(1.0, 0.0, 0.0, 2.0),
    ]

    reverse_execution = build_execution_points(points, reverse=True)

    assert reverse_execution[0].x == pytest.approx(0.0)
    assert reverse_execution[0].theta == pytest.approx(math.pi)
    assert reverse_execution[-1].x == pytest.approx(-1.0)
    assert reverse_execution[-1].theta == pytest.approx(math.pi)


def test_path_follower_reports_success_at_final_pose():
    waypoints = [
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(1.0, 0.0, 0.0),
    ]
    follower = PathFollower(waypoints=waypoints, max_speed_m_s=0.2)

    result = follower.step(Pose2D(1.0, 0.0, 0.0))

    assert result.succeeded is True
    assert result.command.linear_x == pytest.approx(0.0)
    assert result.state == 'SUCCEEDED'
