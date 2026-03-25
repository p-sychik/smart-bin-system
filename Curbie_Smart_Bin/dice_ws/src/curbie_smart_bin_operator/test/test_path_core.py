"""Tests for waypoint replay direction handling."""

import pytest

from curbie_smart_bin_operator.path_core import CommandRecorder
from curbie_smart_bin_operator.path_core import PathFollower
from curbie_smart_bin_operator.path_core import Pose2D
from curbie_smart_bin_operator.path_storage import path_from_payload


def test_path_follower_drives_forward_for_forward_segment():
    follower = PathFollower(
        waypoints=[
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(0.8, 0.0, 0.0),
        ],
        max_speed_m_s=0.2,
    )

    result = follower.step(Pose2D(0.0, 0.0, 0.0))

    assert result.succeeded is False
    assert result.command.linear_x > 0.0


def test_path_follower_preserves_reverse_segment_direction():
    follower = PathFollower(
        waypoints=[
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(-0.8, 0.0, 0.0),
        ],
        max_speed_m_s=0.2,
    )

    result = follower.step(Pose2D(0.0, 0.0, 0.0))

    assert result.succeeded is False
    assert result.command.linear_x < 0.0


def test_command_recorder_preserves_exact_command_sequence_and_duration():
    recorder = CommandRecorder(min_duration_s=0.0)
    recorder.start(0.0, linear_x=0.0, angular_z=0.0)

    recorder.set_command(0.18, 0.0, 0.5)
    recorder.set_command(0.0, -0.5, 1.4)
    recorder.finalize(2.0)

    commands = recorder.commands
    assert len(commands) == 3
    assert commands[0].linear_x == 0.0
    assert commands[0].angular_z == 0.0
    assert commands[0].duration_s == pytest.approx(0.5)
    assert commands[1].linear_x == 0.18
    assert commands[1].angular_z == 0.0
    assert commands[1].duration_s == pytest.approx(0.9)
    assert commands[2].linear_x == 0.0
    assert commands[2].angular_z == -0.5
    assert commands[2].duration_s == pytest.approx(0.6)


def test_path_storage_loads_command_segments_for_motion_steps():
    stored = path_from_payload(
        {
            'path_id': 'demo',
            'display_name': 'Demo',
            'description': '',
            'created_at_s': 1.0,
            'updated_at_s': 1.0,
            'steps': [
                {
                    'step_type': 'motion',
                    'motion': {
                        'commands': [
                            {'linear_x': 0.18, 'angular_z': 0.0, 'duration_s': 0.75},
                            {'linear_x': 0.0, 'angular_z': -0.5, 'duration_s': 0.50},
                        ],
                    },
                }
            ],
        }
    )

    motion = stored.steps[0].motion
    assert motion is not None
    assert len(motion.commands) == 2
    assert motion.commands[0].duration_s == pytest.approx(0.75)
    assert motion.commands[1].angular_z == -0.5
