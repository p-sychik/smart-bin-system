"""Tests for odometry-aware playback helpers."""

import pytest

from turtlebot3_combined_teleop.recording_playback_core import PlaybackCommand
from turtlebot3_combined_teleop.recording_playback_core import Pose2D
from turtlebot3_combined_teleop.recording_playback_core import absolute_pose
from turtlebot3_combined_teleop.recording_playback_core import compute_waypoint_command
from turtlebot3_combined_teleop.recording_playback_core import relative_pose


def test_relative_and_absolute_pose_round_trip():
    anchor = Pose2D(1.0, 2.0, 0.3)
    pose = Pose2D(1.5, 2.2, 0.9)

    relative = relative_pose(anchor, pose)
    restored = absolute_pose(anchor, relative)

    assert restored.x_m == pytest.approx(pose.x_m)
    assert restored.y_m == pytest.approx(pose.y_m)
    assert restored.yaw_rad == pytest.approx(pose.yaw_rad)


def test_waypoint_command_moves_forward_toward_target():
    command = compute_waypoint_command(
        current_pose=Pose2D(0.0, 0.0, 0.0),
        target_pose=Pose2D(1.0, 0.0, 0.0),
    )

    assert isinstance(command, PlaybackCommand)
    assert command.reached is False
    assert command.linear_x > 0.0
    assert command.angular_z == pytest.approx(0.0)


def test_waypoint_command_rotates_to_final_yaw_when_close():
    command = compute_waypoint_command(
        current_pose=Pose2D(1.0, 1.0, 0.0),
        target_pose=Pose2D(1.01, 1.0, 0.5),
        position_tolerance_m=0.05,
        yaw_tolerance_rad=0.05,
        require_yaw_alignment=True,
    )

    assert command.reached is False
    assert command.linear_x == pytest.approx(0.0)
    assert command.angular_z > 0.0


def test_waypoint_command_can_back_into_recorded_target():
    command = compute_waypoint_command(
        current_pose=Pose2D(0.0, 0.0, 0.0),
        target_pose=Pose2D(-0.5, 0.0, 0.0),
        direction_hint=-0.2,
    )

    assert command.reached is False
    assert command.linear_x < 0.0


def test_waypoint_command_accepts_nearby_pose_without_exact_yaw():
    command = compute_waypoint_command(
        current_pose=Pose2D(1.00, 1.00, 0.0),
        target_pose=Pose2D(1.08, 1.04, 0.7),
        position_tolerance_m=0.12,
        require_yaw_alignment=False,
    )

    assert command.reached is True
    assert command.linear_x == pytest.approx(0.0)
    assert command.angular_z == pytest.approx(0.0)
