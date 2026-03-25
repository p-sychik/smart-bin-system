"""Obstacle-aware velocity shaping using the five sonar sensors."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from .path_core import Pose2D
from .path_core import VelocityCommand
from .path_core import wrap_angle


SONAR_LABELS = (
    'right',
    'front_left',
    'left',
    'front_right',
    'front',
)

# Sonar headings in robot frame based on the user's mapping.
SONAR_HEADINGS_RAD = (
    -math.pi / 2.0,
    math.pi / 4.0,
    math.pi / 2.0,
    -math.pi / 4.0,
    0.0,
)


@dataclass(frozen=True)
class ObstacleDecision:
    """Output of the obstacle-aware local planner."""

    command: VelocityCommand
    avoidance_active: bool
    note: str


def _safe_distance(value: float) -> float:
    if value <= 0.0 or math.isnan(value) or math.isinf(value):
        return 5.0
    return value


def compute_obstacle_aware_command(
    current_pose: Pose2D,
    target_pose: Pose2D,
    sonar_distances_m: Sequence[float],
    *,
    max_linear_speed_m_s: float,
    max_angular_speed_rad_s: float,
    waypoint_tolerance_m: float = 0.05,
    obstacle_influence_m: float = 0.60,
    hard_stop_m: float = 0.18,
    attraction_gain: float = 1.2,
    repulsion_gain: float = 0.22,
    angular_gain: float = 1.8,
) -> ObstacleDecision:
    """Blend waypoint following with sonar-based repulsion."""

    dx = target_pose.x - current_pose.x
    dy = target_pose.y - current_pose.y
    distance = math.hypot(dx, dy)
    if distance <= waypoint_tolerance_m:
        yaw_error = wrap_angle(target_pose.theta - current_pose.theta)
        angular = max(
            -max_angular_speed_rad_s,
            min(max_angular_speed_rad_s, angular_gain * yaw_error),
        )
        return ObstacleDecision(
            command=VelocityCommand(0.0, angular if abs(yaw_error) > 0.15 else 0.0),
            avoidance_active=False,
            note='target reached',
        )

    target_heading = wrap_angle(math.atan2(dy, dx) - current_pose.theta)
    attractive_x = attraction_gain * math.cos(target_heading)
    attractive_y = attraction_gain * math.sin(target_heading)

    repulsive_x = 0.0
    repulsive_y = 0.0
    notes = []
    avoidance_active = False
    for distance_m, sonar_heading, label in zip(
        sonar_distances_m,
        SONAR_HEADINGS_RAD,
        SONAR_LABELS,
    ):
        reading = _safe_distance(float(distance_m))
        if reading >= obstacle_influence_m:
            continue
        avoidance_active = True
        strength = repulsion_gain * max(0.0, (1.0 / reading) - (1.0 / obstacle_influence_m))
        repulsive_x -= strength * math.cos(sonar_heading)
        repulsive_y -= strength * math.sin(sonar_heading)
        notes.append(f'{label}:{reading:.2f}m')

    combined_x = attractive_x + repulsive_x
    combined_y = attractive_y + repulsive_y
    desired_heading = math.atan2(combined_y, max(1e-6, combined_x))

    front_min = min(
        _safe_distance(sonar_distances_m[1]),
        _safe_distance(sonar_distances_m[3]),
        _safe_distance(sonar_distances_m[4]),
    )
    if front_min <= hard_stop_m:
        linear = 0.0
        if _safe_distance(sonar_distances_m[2]) > _safe_distance(sonar_distances_m[0]):
            desired_heading = math.pi / 2.0
        else:
            desired_heading = -math.pi / 2.0
        avoidance_active = True
        notes.append(f'hard-stop:{front_min:.2f}m')
    else:
        heading_scale = max(0.0, math.cos(desired_heading))
        linear = min(max_linear_speed_m_s, 0.10 + 0.9 * distance) * heading_scale
        if avoidance_active:
            linear *= 0.7

    angular = max(
        -max_angular_speed_rad_s,
        min(max_angular_speed_rad_s, angular_gain * desired_heading),
    )
    return ObstacleDecision(
        command=VelocityCommand(linear_x=linear, angular_z=angular),
        avoidance_active=avoidance_active,
        note=', '.join(notes) if notes else 'clear',
    )
