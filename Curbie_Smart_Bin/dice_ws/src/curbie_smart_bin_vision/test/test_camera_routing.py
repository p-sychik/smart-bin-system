"""Tests for front/rear camera selection rules."""

import pytest

from curbie_smart_bin_vision.camera_routing import active_camera_name
from curbie_smart_bin_vision.camera_routing import camera_for_align_mode
from curbie_smart_bin_vision.camera_routing import camera_for_manual_seek
from curbie_smart_bin_vision.camera_routing import marker_x_sign_for_camera
from curbie_smart_bin_vision.camera_routing import normalize_camera_name


def test_normalize_camera_name_falls_back_to_default():
    assert normalize_camera_name('front') == 'front'
    assert normalize_camera_name('rear') == 'rear'
    assert normalize_camera_name('side', default='rear') == 'rear'


def test_align_mode_uses_front_for_pickup_and_rear_for_dropoff():
    assert camera_for_align_mode('pickup', 'front', 'rear') == 'front'
    assert camera_for_align_mode('dropoff', 'front', 'rear') == 'rear'


def test_align_mode_rejects_unknown_values():
    with pytest.raises(ValueError):
        camera_for_align_mode('tow', 'front', 'rear')


def test_manual_seek_prefers_rear_when_hooked():
    assert (
        camera_for_manual_seek(
            'front',
            hook_attached=False,
            use_rear_camera_when_hooked=True,
        )
        == 'front'
    )
    assert (
        camera_for_manual_seek(
            'front',
            hook_attached=True,
            use_rear_camera_when_hooked=True,
        )
        == 'rear'
    )
    assert (
        camera_for_manual_seek(
            'rear',
            hook_attached=False,
            use_rear_camera_when_hooked=False,
        )
        == 'rear'
    )


def test_active_camera_prefers_align_override_over_manual_seek():
    assert active_camera_name(
        align_camera_name='rear',
        manual_seek_active=True,
        manual_camera_name='front',
        hook_attached=False,
        use_rear_camera_when_hooked=True,
    ) == 'rear'
    assert active_camera_name(
        align_camera_name=None,
        manual_seek_active=True,
        manual_camera_name='front',
        hook_attached=True,
        use_rear_camera_when_hooked=True,
    ) == 'rear'
    assert active_camera_name(
        align_camera_name=None,
        manual_seek_active=False,
        manual_camera_name='front',
        hook_attached=False,
        use_rear_camera_when_hooked=True,
    ) is None


def test_marker_x_sign_can_differ_between_front_and_rear_cameras():
    assert marker_x_sign_for_camera('front', 1.0, -1.0) == 1.0
    assert marker_x_sign_for_camera('rear', 1.0, -1.0) == -1.0
    assert marker_x_sign_for_camera('front', 0.0, -1.0) == -1.0
