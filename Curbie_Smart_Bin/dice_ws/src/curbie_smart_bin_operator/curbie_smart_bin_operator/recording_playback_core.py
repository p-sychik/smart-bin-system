"""Pure helpers for odometry-aware recording playback."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    """Planar robot pose."""

    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class PlaybackCommand:
    """Closed-loop command for driving toward a recorded waypoint."""

    linear_x: float
    angular_z: float
    reached: bool


def clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp a value to the provided bounds."""

    return max(min_value, min(max_value, value))


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi]."""

    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def relative_pose(anchor_pose: Pose2D, pose: Pose2D) -> Pose2D:
    """Express a world pose in the anchor pose frame."""

    dx = pose.x_m - anchor_pose.x_m
    dy = pose.y_m - anchor_pose.y_m
    cos_yaw = math.cos(anchor_pose.yaw_rad)
    sin_yaw = math.sin(anchor_pose.yaw_rad)
    return Pose2D(
        x_m=(cos_yaw * dx) + (sin_yaw * dy),
        y_m=(-sin_yaw * dx) + (cos_yaw * dy),
        yaw_rad=wrap_angle(pose.yaw_rad - anchor_pose.yaw_rad),
    )


def absolute_pose(anchor_pose: Pose2D, relative_target: Pose2D) -> Pose2D:
    """Project a relative pose from the anchor frame into the world frame."""

    cos_yaw = math.cos(anchor_pose.yaw_rad)
    sin_yaw = math.sin(anchor_pose.yaw_rad)
    return Pose2D(
        x_m=anchor_pose.x_m + (cos_yaw * relative_target.x_m) - (
            sin_yaw * relative_target.y_m
        ),
        y_m=anchor_pose.y_m + (sin_yaw * relative_target.x_m) + (
            cos_yaw * relative_target.y_m
        ),
        yaw_rad=wrap_angle(anchor_pose.yaw_rad + relative_target.yaw_rad),
    )


def compute_waypoint_command(
    current_pose: Pose2D,
    target_pose: Pose2D,
    *,
    direction_hint: float = 0.0,
    position_tolerance_m: float = 0.12,
    yaw_tolerance_rad: float = 0.20,
    require_yaw_alignment: bool = False,
    position_kp: float = 0.9,
    heading_kp: float = 1.8,
    yaw_kp: float = 1.8,
    max_linear_speed_m_s: float = 0.18,
    max_angular_speed_rad_s: float = 0.8,
    heading_stop_threshold_rad: float = 0.45,
) -> PlaybackCommand:
    """Drive toward a target pose while preserving the recorded final destination."""

    dx = target_pose.x_m - current_pose.x_m
    dy = target_pose.y_m - current_pose.y_m
    distance_m = math.hypot(dx, dy)

    if distance_m <= position_tolerance_m:
        if not require_yaw_alignment:
            return PlaybackCommand(0.0, 0.0, True)
        yaw_error = wrap_angle(target_pose.yaw_rad - current_pose.yaw_rad)
        if abs(yaw_error) <= yaw_tolerance_rad:
            return PlaybackCommand(0.0, 0.0, True)
        return PlaybackCommand(
            linear_x=0.0,
            angular_z=clamp(
                yaw_kp * yaw_error,
                -max_angular_speed_rad_s,
                max_angular_speed_rad_s,
            ),
            reached=False,
        )

    travel_heading = math.atan2(dy, dx)
    linear_sign = 1.0
    if direction_hint < -0.05:
        travel_heading = wrap_angle(travel_heading + math.pi)
        linear_sign = -1.0

    heading_error = wrap_angle(travel_heading - current_pose.yaw_rad)
    angular_z = clamp(
        heading_kp * heading_error,
        -max_angular_speed_rad_s,
        max_angular_speed_rad_s,
    )

    if abs(heading_error) >= heading_stop_threshold_rad:
        linear_x = 0.0
    else:
        heading_scale = max(
            0.15,
            1.0 - (abs(heading_error) / max(heading_stop_threshold_rad, 1e-6)),
        )
        linear_x = linear_sign * min(
            max_linear_speed_m_s,
            position_kp * distance_m,
        ) * heading_scale

    return PlaybackCommand(linear_x=linear_x, angular_z=angular_z, reached=False)
