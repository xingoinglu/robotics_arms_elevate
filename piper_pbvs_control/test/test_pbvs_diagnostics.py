"""Tests for PBVS diagnostic-only helpers."""

from moveit_msgs.action import MoveGroup
from trajectory_msgs.msg import JointTrajectoryPoint

from piper_pbvs_control.pbvs_controller import PiperPbvsController


def _set_final_point(robot_trajectory, names, positions):
    trajectory = robot_trajectory.joint_trajectory
    trajectory.joint_names = list(names)
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    trajectory.points = [point]


def test_final_target_prefers_executed_trajectory_and_reorders_joints():
    result = MoveGroup.Result()
    _set_final_point(
        result.executed_trajectory,
        ['joint3', 'joint1', 'joint6', 'joint2', 'joint5', 'joint4'],
        [0.3, 0.1, 0.6, 0.2, 0.5, 0.4],
    )
    _set_final_point(
        result.planned_trajectory,
        PiperPbvsController.ARM_JOINT_NAMES,
        [1.0] * 6,
    )

    target = PiperPbvsController._final_moveit_arm_target(result)

    assert target == (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)


def test_final_target_falls_back_to_planned_trajectory():
    result = MoveGroup.Result()
    expected = (-0.1, 0.2, -0.3, 0.4, -0.5, 0.6)
    _set_final_point(
        result.planned_trajectory,
        PiperPbvsController.ARM_JOINT_NAMES,
        expected,
    )

    target = PiperPbvsController._final_moveit_arm_target(result)

    assert target == expected


def test_final_target_returns_none_for_incomplete_joint_data():
    result = MoveGroup.Result()
    _set_final_point(
        result.executed_trajectory,
        ['joint1', 'joint2'],
        [0.1, 0.2],
    )

    assert PiperPbvsController._final_moveit_arm_target(result) is None


def test_arm_joint_formatter_includes_names_and_radian_values():
    text = PiperPbvsController._format_arm_joints(
        (0.1, -0.2, 0.3, -0.4, 0.5, -0.6),
    )

    assert 'joint1=+0.100000' in text
    assert 'joint2=-0.200000' in text
    assert 'joint6=-0.600000' in text
