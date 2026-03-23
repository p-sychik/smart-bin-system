"""Pure helpers for recording and executing odometry paths."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


@dataclass(frozen=True)
class Pose2D:
    """2D pose in a local frame."""

    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class VelocityCommand:
    """Velocity command produced by the path follower."""

    linear_x: float
    angular_z: float


@dataclass(frozen=True)
class PathPointRecord:
    """Recorded relative path sample."""

    x: float
    y: float
    theta: float
    timestamp: float


@dataclass(frozen=True)
class PathStepResult:
    """Follower output plus high-level progress."""

    command: VelocityCommand
    current_waypoint: int
    total_waypoints: int
    distance_remaining: float
    progress_percent: float
    succeeded: bool
    state: str


def pose_compose(base_pose: Pose2D, delta_pose: Pose2D) -> Pose2D:
    """Compose a base pose with a delta pose."""
    cos_t = math.cos(base_pose.theta)
    sin_t = math.sin(base_pose.theta)
    x = base_pose.x + cos_t * delta_pose.x - sin_t * delta_pose.y
    y = base_pose.y + sin_t * delta_pose.x + cos_t * delta_pose.y
    theta = wrap_angle(base_pose.theta + delta_pose.theta)
    return Pose2D(x=x, y=y, theta=theta)


def pose_relative(reference_pose: Pose2D, target_pose: Pose2D) -> Pose2D:
    """Express a target pose in the reference pose frame."""
    dx = target_pose.x - reference_pose.x
    dy = target_pose.y - reference_pose.y
    cos_t = math.cos(reference_pose.theta)
    sin_t = math.sin(reference_pose.theta)
    x = cos_t * dx + sin_t * dy
    y = -sin_t * dx + cos_t * dy
    theta = wrap_angle(target_pose.theta - reference_pose.theta)
    return Pose2D(x=x, y=y, theta=theta)


def path_point_record_to_pose(point: PathPointRecord) -> Pose2D:
    """Convert a recorded point to a pose."""
    return Pose2D(x=point.x, y=point.y, theta=point.theta)


class PathRecorder:
    """Record a relative path with threshold-based downsampling."""

    def __init__(
        self,
        distance_threshold_m: float = 0.05,
        yaw_threshold_rad: float = math.radians(5.0),
    ) -> None:
        self.distance_threshold_m = max(0.0, distance_threshold_m)
        self.yaw_threshold_rad = max(0.0, yaw_threshold_rad)
        self._points: List[PathPointRecord] = []
        self.total_distance_m = 0.0

    @property
    def points(self) -> List[PathPointRecord]:
        return list(self._points)

    def start(self, start_timestamp_s: float) -> None:
        self._points = [PathPointRecord(0.0, 0.0, 0.0, start_timestamp_s)]
        self.total_distance_m = 0.0

    def add_pose(self, pose: Pose2D, timestamp_s: float) -> bool:
        """Add a relative pose if it exceeds distance/yaw thresholds."""
        if not self._points:
            self.start(timestamp_s)
            return True

        last = self._points[-1]
        dx = pose.x - last.x
        dy = pose.y - last.y
        dtheta = wrap_angle(pose.theta - last.theta)
        distance = math.hypot(dx, dy)
        if distance < self.distance_threshold_m and abs(dtheta) < self.yaw_threshold_rad:
            return False

        self.total_distance_m += distance
        self._points.append(
            PathPointRecord(
                x=pose.x,
                y=pose.y,
                theta=pose.theta,
                timestamp=timestamp_s,
            )
        )
        return True

    def finalize(self, pose: Pose2D, timestamp_s: float) -> None:
        """Ensure the final pose is included in the recording."""
        if not self._points:
            self.start(timestamp_s)
            return

        last = self._points[-1]
        dx = pose.x - last.x
        dy = pose.y - last.y
        dtheta = wrap_angle(pose.theta - last.theta)
        if abs(dx) < 1e-9 and abs(dy) < 1e-9 and abs(dtheta) < 1e-9:
            return

        self.total_distance_m += math.hypot(dx, dy)
        self._points.append(
            PathPointRecord(
                x=pose.x,
                y=pose.y,
                theta=pose.theta,
                timestamp=timestamp_s,
            )
        )


def build_execution_points(
    points: Sequence[PathPointRecord],
    reverse: bool,
) -> List[Pose2D]:
    """Convert stored path points into an execution path for the current run."""
    poses = [path_point_record_to_pose(point) for point in points]
    if not reverse or not poses:
        return poses

    end_pose = poses[-1]
    reversed_poses: List[Pose2D] = []
    for pose in reversed(poses):
        relative_pose = pose_relative(end_pose, pose)
        reversed_poses.append(
            Pose2D(
                x=relative_pose.x,
                y=relative_pose.y,
                theta=wrap_angle(relative_pose.theta + math.pi),
            )
        )
    return reversed_poses


class PathFollower:
    """Simple waypoint follower in a start-relative frame."""

    def __init__(
        self,
        waypoints: Sequence[Pose2D],
        max_speed_m_s: float,
        waypoint_tolerance_m: float = 0.05,
        final_position_tolerance_m: float = 0.04,
        final_heading_tolerance_rad: float = 0.15,
        max_angular_speed_rad_s: float = 0.9,
        linear_kp: float = 0.8,
        angular_kp: float = 1.6,
        heading_mix: float = 0.35,
    ) -> None:
        self.waypoints = list(waypoints)
        self.max_speed_m_s = max(0.02, max_speed_m_s)
        self.waypoint_tolerance_m = max(0.01, waypoint_tolerance_m)
        self.final_position_tolerance_m = max(0.01, final_position_tolerance_m)
        self.final_heading_tolerance_rad = max(0.02, final_heading_tolerance_rad)
        self.max_angular_speed_rad_s = max(0.1, max_angular_speed_rad_s)
        self.linear_kp = max(0.1, linear_kp)
        self.angular_kp = max(0.1, angular_kp)
        self.heading_mix = max(0.0, min(1.0, heading_mix))
        self.current_index = 1 if len(self.waypoints) > 1 else 0

    def _distance_remaining(self, current_pose: Pose2D) -> float:
        if not self.waypoints:
            return 0.0
        index = min(self.current_index, len(self.waypoints) - 1)
        target = self.waypoints[index]
        total = math.hypot(target.x - current_pose.x, target.y - current_pose.y)
        for i in range(index, len(self.waypoints) - 1):
            a = self.waypoints[i]
            b = self.waypoints[i + 1]
            total += math.hypot(b.x - a.x, b.y - a.y)
        return total

    def step(self, current_pose: Pose2D) -> PathStepResult:
        """Advance the controller by one step."""
        if not self.waypoints:
            return PathStepResult(
                command=VelocityCommand(0.0, 0.0),
                current_waypoint=0,
                total_waypoints=0,
                distance_remaining=0.0,
                progress_percent=100.0,
                succeeded=True,
                state='SUCCEEDED',
            )

        last_index = len(self.waypoints) - 1
        while self.current_index < last_index:
            target = self.waypoints[self.current_index]
            distance = math.hypot(target.x - current_pose.x, target.y - current_pose.y)
            if distance > self.waypoint_tolerance_m:
                break
            self.current_index += 1

        target = self.waypoints[self.current_index]
        dx = target.x - current_pose.x
        dy = target.y - current_pose.y
        distance = math.hypot(dx, dy)
        bearing_error = wrap_angle(math.atan2(dy, dx) - current_pose.theta)
        heading_error = wrap_angle(target.theta - current_pose.theta)

        is_final = self.current_index == last_index
        if is_final:
            if (
                distance <= self.final_position_tolerance_m
                and abs(heading_error) <= self.final_heading_tolerance_rad
            ):
                return PathStepResult(
                    command=VelocityCommand(0.0, 0.0),
                    current_waypoint=self.current_index,
                    total_waypoints=len(self.waypoints),
                    distance_remaining=0.0,
                    progress_percent=100.0,
                    succeeded=True,
                    state='SUCCEEDED',
                )

        angular = (
            self.angular_kp * bearing_error
            + self.heading_mix * heading_error
        )
        angular = max(
            -self.max_angular_speed_rad_s,
            min(self.max_angular_speed_rad_s, angular),
        )

        heading_gate = 0.8
        if abs(bearing_error) >= heading_gate:
            linear = 0.0
        else:
            linear_scale = max(0.2, 1.0 - abs(bearing_error) / heading_gate)
            linear = min(
                self.max_speed_m_s,
                max(0.04, self.linear_kp * distance),
            ) * linear_scale
            if is_final and abs(heading_error) > 0.3:
                linear = min(linear, 0.05)

        distance_remaining = self._distance_remaining(current_pose)
        progress_percent = 0.0
        if last_index > 0:
            progress_percent = 100.0 * (self.current_index / last_index)

        return PathStepResult(
            command=VelocityCommand(linear_x=linear, angular_z=angular),
            current_waypoint=self.current_index,
            total_waypoints=len(self.waypoints),
            distance_remaining=distance_remaining,
            progress_percent=progress_percent,
            succeeded=False,
            state='FOLLOWING',
        )


def points_from_dicts(raw_points: Iterable[dict]) -> List[PathPointRecord]:
    """Create typed path points from JSON dictionaries."""
    return [
        PathPointRecord(
            x=float(point['x']),
            y=float(point['y']),
            theta=float(point['theta']),
            timestamp=float(point['timestamp']),
        )
        for point in raw_points
    ]
