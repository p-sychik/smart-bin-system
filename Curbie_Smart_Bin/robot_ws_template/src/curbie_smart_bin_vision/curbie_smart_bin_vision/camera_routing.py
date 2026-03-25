"""Helpers for choosing the active camera feed for marker vision."""

from __future__ import annotations

from typing import Optional


VALID_CAMERA_NAMES = {'front', 'rear'}


def normalize_camera_name(name: str, default: str = 'front') -> str:
    """Normalize a configured camera name, falling back safely."""
    normalized_default = (default or 'front').strip().lower()
    if normalized_default not in VALID_CAMERA_NAMES:
        normalized_default = 'front'

    normalized = (name or '').strip().lower()
    if normalized in VALID_CAMERA_NAMES:
        return normalized
    return normalized_default


def camera_for_align_mode(
    mode: str,
    pickup_camera_name: str,
    dropoff_camera_name: str,
) -> str:
    """Return the camera to use for a mission alignment mode."""
    normalized_mode = (mode or '').strip().lower()
    if normalized_mode == 'pickup':
        return normalize_camera_name(pickup_camera_name, default='front')
    if normalized_mode == 'dropoff':
        return normalize_camera_name(dropoff_camera_name, default='rear')
    raise ValueError(f'Unsupported alignment mode: {mode!r}')


def camera_for_manual_seek(
    manual_camera_name: str,
    hook_attached: bool,
    use_rear_camera_when_hooked: bool,
) -> str:
    """Return the active manual-seek camera based on hook state."""
    if hook_attached and use_rear_camera_when_hooked:
        return 'rear'
    return normalize_camera_name(manual_camera_name, default='front')


def active_camera_name(
    align_camera_name: Optional[str],
    manual_seek_active: bool,
    manual_camera_name: str,
    hook_attached: bool,
    use_rear_camera_when_hooked: bool,
) -> Optional[str]:
    """Return the currently active vision camera, if any."""
    if align_camera_name:
        return normalize_camera_name(align_camera_name, default='front')
    if manual_seek_active:
        return camera_for_manual_seek(
            manual_camera_name=manual_camera_name,
            hook_attached=hook_attached,
            use_rear_camera_when_hooked=use_rear_camera_when_hooked,
        )
    return None


def marker_x_sign_for_camera(
    camera_name: str,
    front_marker_x_sign: float,
    rear_marker_x_sign: float,
) -> float:
    """Return the lateral sign correction for the selected camera."""
    normalized_name = normalize_camera_name(camera_name, default='front')
    sign = front_marker_x_sign if normalized_name == 'front' else rear_marker_x_sign
    if abs(sign) < 1e-6:
        return -1.0
    return 1.0 if sign > 0.0 else -1.0
