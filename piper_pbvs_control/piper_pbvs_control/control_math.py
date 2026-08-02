"""Pure pose mathematics for PBVS control."""

import math

import numpy as np


def normalize_quaternion(quaternion):
    """Return a unit ``[x, y, z, w]`` quaternion."""
    quaternion = np.asarray(quaternion, dtype=np.float64).reshape(4)
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12 or not np.isfinite(norm):
        raise ValueError('quaternion must be finite and non-zero')
    return quaternion / norm


def quaternion_to_matrix(quaternion):
    """Convert an ``[x, y, z, w]`` quaternion to a rotation matrix."""
    x_value, y_value, z_value, w_value = normalize_quaternion(quaternion)
    return np.array([
        [
            1.0 - 2.0 * (y_value ** 2 + z_value ** 2),
            2.0 * (x_value * y_value - z_value * w_value),
            2.0 * (x_value * z_value + y_value * w_value),
        ],
        [
            2.0 * (x_value * y_value + z_value * w_value),
            1.0 - 2.0 * (x_value ** 2 + z_value ** 2),
            2.0 * (y_value * z_value - x_value * w_value),
        ],
        [
            2.0 * (x_value * z_value - y_value * w_value),
            2.0 * (y_value * z_value + x_value * w_value),
            1.0 - 2.0 * (x_value ** 2 + y_value ** 2),
        ],
    ])


def matrix_to_quaternion(rotation):
    """Convert a rotation matrix to a unit ``[x, y, z, w]`` quaternion."""
    rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = np.trace(rotation)
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array([
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(
                1.0 + rotation[0, 0]
                - rotation[1, 1] - rotation[2, 2]
            ) * 2.0
            quaternion = np.array([
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
            ])
        elif index == 1:
            scale = math.sqrt(
                1.0 + rotation[1, 1]
                - rotation[0, 0] - rotation[2, 2]
            ) * 2.0
            quaternion = np.array([
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
            ])
        else:
            scale = math.sqrt(
                1.0 + rotation[2, 2]
                - rotation[0, 0] - rotation[1, 1]
            ) * 2.0
            quaternion = np.array([
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ])
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return normalize_quaternion(quaternion)


def rotation_vector(desired_quaternion, current_quaternion):
    """Return the base-frame rotation vector from current to desired."""
    desired = quaternion_to_matrix(desired_quaternion)
    current = quaternion_to_matrix(current_quaternion)
    error_rotation = desired @ current.T
    cosine = np.clip((np.trace(error_rotation) - 1.0) * 0.5, -1.0, 1.0)
    angle = math.acos(cosine)
    if angle < 1e-9:
        return np.zeros(3)
    if math.pi - angle < 1e-5:
        eigenvalues, eigenvectors = np.linalg.eig(error_rotation)
        axis = np.real(eigenvectors[:, np.argmin(np.abs(eigenvalues - 1.0))])
        axis /= np.linalg.norm(axis)
        return axis * angle
    axis = np.array([
        error_rotation[2, 1] - error_rotation[1, 2],
        error_rotation[0, 2] - error_rotation[2, 0],
        error_rotation[1, 0] - error_rotation[0, 1],
    ]) / (2.0 * math.sin(angle))
    return axis * angle


def rotation_from_vector(vector):
    """Return a rotation matrix from an axis-angle vector."""
    vector = np.asarray(vector, dtype=np.float64).reshape(3)
    angle = np.linalg.norm(vector)
    if angle < 1e-12:
        return np.eye(3)
    axis = vector / angle
    cross = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return (
        np.eye(3)
        + math.sin(angle) * cross
        + (1.0 - math.cos(angle)) * (cross @ cross)
    )


def align_tool_z_preserve_roll(reference_quaternion, target_axis):
    """Align tool +Z by the shortest rotation without adding tool roll."""
    reference_rotation = quaternion_to_matrix(reference_quaternion)
    current_axis = reference_rotation[:, 2]
    target_axis = np.asarray(target_axis, dtype=np.float64).reshape(3)
    target_norm = np.linalg.norm(target_axis)
    if target_norm < 1e-12 or not np.isfinite(target_norm):
        raise ValueError('target_axis must be finite and non-zero')
    target_axis /= target_norm

    cross_axis = np.cross(current_axis, target_axis)
    sine = np.linalg.norm(cross_axis)
    cosine = float(np.clip(np.dot(current_axis, target_axis), -1.0, 1.0))
    if sine < 1e-9:
        if cosine > 0.0:
            alignment = np.eye(3)
        else:
            alignment = rotation_from_vector(
                reference_rotation[:, 0] * math.pi
            )
    else:
        angle = math.atan2(sine, cosine)
        alignment = rotation_from_vector(cross_axis / sine * angle)

    return matrix_to_quaternion(alignment @ reference_rotation)


def offset_along_press_axis(position, quaternion, offset):
    """Offset a position along the pose's tool +Z press axis."""
    position = np.asarray(position, dtype=np.float64).reshape(3)
    press_axis = quaternion_to_matrix(quaternion)[:, 2]
    return position + float(offset) * press_axis


def offset_along_panel_horizontal(position, button_quaternion, offset):
    """Offset along button-frame +X; positive is left facing the panel."""
    position = np.asarray(position, dtype=np.float64).reshape(3)
    horizontal_axis = quaternion_to_matrix(button_quaternion)[:, 0]
    return position + float(offset) * horizontal_axis


def coarse_standoff_errors(
    button_position,
    tcp_position,
    control_quaternion,
    desired_standoff,
):
    """Decompose coarse TCP error into panel-normal and lateral parts."""
    button_position = np.asarray(
        button_position,
        dtype=np.float64,
    ).reshape(3)
    tcp_position = np.asarray(tcp_position, dtype=np.float64).reshape(3)
    press_axis = quaternion_to_matrix(control_quaternion)[:, 2]
    button_from_tcp = button_position - tcp_position
    axial_distance = float(np.dot(button_from_tcp, press_axis))
    lateral_vector = button_from_tcp - axial_distance * press_axis
    lateral_error = float(np.linalg.norm(lateral_vector))
    axial_error = axial_distance - float(desired_standoff)
    return axial_distance, axial_error, lateral_vector, lateral_error


def signed_normal_drift(reference_position, new_position, quaternion):
    """Project a target-position change onto the locked press normal."""
    reference = np.asarray(reference_position, dtype=np.float64).reshape(3)
    new = np.asarray(new_position, dtype=np.float64).reshape(3)
    press_axis = quaternion_to_matrix(quaternion)[:, 2]
    return float(np.dot(new - reference, press_axis))


def pose_error(target_position, target_quaternion, current_position,
               current_quaternion):
    """Return translation vector, rotation vector, and their magnitudes."""
    translation = (
        np.asarray(target_position, dtype=np.float64)
        - np.asarray(current_position, dtype=np.float64)
    )
    rotation = rotation_vector(target_quaternion, current_quaternion)
    return translation, rotation, np.linalg.norm(translation), np.linalg.norm(
        rotation
    )


def limited_pose_step(
    current_position,
    current_quaternion,
    target_position,
    target_quaternion,
    translation_gain,
    rotation_gain,
    max_translation,
    max_rotation,
):
    """Apply bounded proportional translation and rotation pose steps."""
    translation, rotation, _, _ = pose_error(
        target_position,
        target_quaternion,
        current_position,
        current_quaternion,
    )
    translation *= translation_gain
    rotation *= rotation_gain
    translation_norm = np.linalg.norm(translation)
    rotation_norm = np.linalg.norm(rotation)
    if translation_norm > max_translation:
        translation *= max_translation / translation_norm
    if rotation_norm > max_rotation:
        rotation *= max_rotation / rotation_norm

    new_position = np.asarray(current_position) + translation
    current_rotation = quaternion_to_matrix(current_quaternion)
    new_rotation = rotation_from_vector(rotation) @ current_rotation
    return new_position, matrix_to_quaternion(new_rotation)


def tcp_to_flange_pose(tcp_position, tcp_quaternion, tcp_offset):
    """Convert a desired TCP pose to the Piper J6/flange command pose."""
    tcp_position = np.asarray(tcp_position, dtype=np.float64).reshape(3)
    tcp_offset = np.asarray(tcp_offset, dtype=np.float64).reshape(3)
    rotation = quaternion_to_matrix(tcp_quaternion)
    flange_position = tcp_position - rotation @ tcp_offset
    return flange_position, normalize_quaternion(tcp_quaternion)


def quaternion_to_euler_xyz(quaternion):
    """Return XYZ fixed-axis roll, pitch, yaw in radians."""
    rotation = quaternion_to_matrix(quaternion)
    pitch = math.asin(np.clip(-rotation[2, 0], -1.0, 1.0))
    if abs(math.cos(pitch)) > 1e-7:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    return np.array([roll, pitch, yaw])


def average_stable_poses(
    positions,
    quaternions,
    max_position_spread,
    max_angle_spread,
):
    """Average poses if all samples lie within configured spreads."""
    positions = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
    quaternions = np.asarray(quaternions, dtype=np.float64).reshape(-1, 4)
    if positions.shape[0] == 0 or positions.shape[0] != quaternions.shape[0]:
        return None
    reference = normalize_quaternion(quaternions[0])
    aligned = []
    for quaternion in quaternions:
        quaternion = normalize_quaternion(quaternion)
        if np.dot(quaternion, reference) < 0.0:
            quaternion = -quaternion
        aligned.append(quaternion)
    average_position = np.mean(positions, axis=0)
    average_quaternion = normalize_quaternion(np.mean(aligned, axis=0))
    position_spread = np.max(
        np.linalg.norm(positions - average_position, axis=1)
    )
    angle_spread = max(
        np.linalg.norm(rotation_vector(average_quaternion, quaternion))
        for quaternion in aligned
    )
    if (
        position_spread > max_position_spread
        or angle_spread > max_angle_spread
    ):
        return None
    return average_position, average_quaternion
