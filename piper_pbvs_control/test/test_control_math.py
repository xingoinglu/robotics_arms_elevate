"""Unit tests for PBVS pose mathematics."""

import math

import numpy as np

from piper_pbvs_control.control_math import (
    align_tool_z_preserve_roll,
    average_stable_poses,
    coarse_standoff_errors,
    limited_pose_step,
    matrix_to_quaternion,
    offset_along_press_axis,
    pose_error,
    quaternion_to_euler_xyz,
    quaternion_to_matrix,
    rotation_vector,
    signed_normal_drift,
    tcp_to_flange_pose,
)


def test_align_tool_z_keeps_pose_when_axis_is_already_aligned():
    """An aligned reference pose must not acquire an extra tool roll."""
    reference = matrix_to_quaternion(np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ]))

    result = align_tool_z_preserve_roll(reference, [0.0, 0.0, 1.0])

    assert np.allclose(
        quaternion_to_matrix(result),
        quaternion_to_matrix(reference),
        atol=1e-9,
    )


def test_align_tool_z_uses_shortest_rotation():
    """A perpendicular press axis must need only a 90 degree swing."""
    reference = matrix_to_quaternion(np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ]))

    result = align_tool_z_preserve_roll(reference, [1.0, 0.0, 0.0])
    rotation = quaternion_to_matrix(result)
    angular_change = np.linalg.norm(rotation_vector(result, reference))

    assert np.allclose(rotation[:, 2], [1.0, 0.0, 0.0], atol=1e-9)
    assert np.isclose(angular_change, math.pi / 2.0)


def test_align_tool_z_handles_opposite_axis_deterministically():
    """An opposite axis must use the reference tool X as the flip axis."""
    reference = np.array([0.0, 0.0, 0.0, 1.0])

    first = align_tool_z_preserve_roll(reference, [0.0, 0.0, -1.0])
    second = align_tool_z_preserve_roll(reference, [0.0, 0.0, -1.0])
    rotation = quaternion_to_matrix(first)

    assert np.allclose(rotation[:, 2], [0.0, 0.0, -1.0], atol=1e-9)
    assert np.allclose(rotation[:, 0], [1.0, 0.0, 0.0], atol=1e-9)
    assert np.allclose(first, second)


def test_align_tool_z_changes_continuously_for_nearby_axes():
    """Nearby observed normals must not introduce a roll discontinuity."""
    reference = matrix_to_quaternion(np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ]))
    first_axis = np.array([0.20, -0.10, 1.0])
    second_axis = np.array([0.201, -0.099, 1.0])
    first_axis /= np.linalg.norm(first_axis)
    second_axis /= np.linalg.norm(second_axis)

    first = align_tool_z_preserve_roll(reference, first_axis)
    second = align_tool_z_preserve_roll(reference, second_axis)

    assert np.allclose(
        quaternion_to_matrix(first)[:, 2],
        first_axis,
        atol=1e-9,
    )
    assert np.allclose(
        quaternion_to_matrix(second)[:, 2],
        second_axis,
        atol=1e-9,
    )
    assert np.linalg.norm(rotation_vector(second, first)) < 0.002


def test_align_tool_z_rejects_invalid_target_axis():
    """A missing press direction must not produce a motion pose."""
    with np.testing.assert_raises(ValueError):
        align_tool_z_preserve_roll(
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
        )


def test_tcp_to_flange_removes_local_tool_offset():
    """A TCP target must be converted to the SDK's J6 pose."""
    quaternion = np.array([0.0, 0.0, 0.0, 1.0])
    position, result_quaternion = tcp_to_flange_pose(
        [0.4, 0.1, 0.3],
        quaternion,
        [0.0, 0.0, 0.1358],
    )

    assert np.allclose(position, [0.4, 0.1, 0.1642])
    assert np.allclose(result_quaternion, quaternion)


def test_offset_uses_tool_positive_z_press_axis():
    """Negative standoff must move away from the panel along tool Z."""
    result = offset_along_press_axis(
        [0.4, 0.1, 0.3],
        [0.0, 0.0, 0.0, 1.0],
        -0.08,
    )

    assert np.allclose(result, [0.4, 0.1, 0.22])


def test_coarse_errors_use_button_local_normal_and_tangent_plane():
    """Coarse XY/axial acceptance must rotate with the button frame."""
    control_quaternion = matrix_to_quaternion(np.array([
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ]))

    axial_distance, axial_error, lateral, lateral_error = (
        coarse_standoff_errors(
            [0.08, 0.004, 0.0],
            [0.0, 0.0, 0.0],
            control_quaternion,
            0.08,
        )
    )

    assert np.isclose(axial_distance, 0.08)
    assert np.isclose(axial_error, 0.0)
    assert np.allclose(lateral, [0.0, 0.004, 0.0])
    assert np.isclose(lateral_error, 0.004)


def test_coarse_axial_error_reports_distance_from_requested_standoff():
    """Positive axial error means TCP is farther from the button."""
    _, axial_error, _, lateral_error = coarse_standoff_errors(
        [0.0, 0.0, 0.091],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        0.08,
    )

    assert np.isclose(axial_error, 0.011)
    assert np.isclose(lateral_error, 0.0)


def test_signed_normal_drift_ignores_panel_lateral_motion():
    """The B1 guard must only reject changes along the panel normal."""
    quaternion = matrix_to_quaternion(np.array([
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ]))

    drift = signed_normal_drift(
        [0.4, 0.0, 0.2],
        [0.43, 0.05, 0.18],
        quaternion,
    )

    assert np.isclose(drift, 0.03)


def test_pose_step_enforces_translation_and_rotation_limits():
    """PBVS proportional steps must never exceed configured maxima."""
    target_quaternion = matrix_to_quaternion(np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ]))
    position, quaternion = limited_pose_step(
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        target_quaternion,
        1.0,
        1.0,
        0.002,
        math.radians(2.0),
    )
    _, _, translation_error, angular_error = pose_error(
        position,
        quaternion,
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    )

    assert translation_error <= 0.002 + 1e-12
    assert angular_error <= math.radians(2.0) + 1e-12


def test_pose_stability_handles_quaternion_sign():
    """Equivalent positive and negative quaternions must average together."""
    result = average_stable_poses(
        [[0.4, 0.1, 0.3], [0.401, 0.1, 0.3]],
        [[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, -1.0]],
        0.003,
        math.radians(3.0),
    )

    assert result is not None
    position, quaternion = result
    assert np.allclose(position, [0.4005, 0.1, 0.3])
    assert np.allclose(quaternion_to_matrix(quaternion), np.eye(3))


def test_quaternion_to_euler_xyz_round_trip_axes():
    """Euler conversion must match Piper's XYZ feedback convention."""
    rotation = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    euler = quaternion_to_euler_xyz(matrix_to_quaternion(rotation))

    assert np.allclose(euler, [0.0, 0.0, math.pi / 2.0])
