"""Vision-guided MoveIt coarse positioning for elevator buttons."""

from collections import deque
import copy
import math
import threading
import time

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene
import numpy as np
from piper_msgs.action import PressButton
from piper_msgs.msg import PiperStatusMsg
from piper_msgs.srv import SetInterest
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from piper_pbvs_control.control_math import (
    align_tool_z_preserve_roll,
    average_stable_poses,
    coarse_pose_is_acceptable,
    coarse_standoff_errors,
    coarse_total_attempts,
    offset_along_panel_horizontal,
    offset_along_press_axis,
    pose_error,
    quaternion_to_matrix,
    translated_base_x,
    x_distance_metres,
)


class TaskFailure(RuntimeError):
    """Raised when a guarded coarse-positioning task cannot continue."""


class TaskCanceled(RuntimeError):
    """Raised when the client cancels an active positioning task."""


class PiperPbvsController(Node):
    """Coordinate perception and guarded MoveIt coarse positioning."""

    ARM_JOINT_NAMES = tuple(f'joint{index}' for index in range(1, 7))
    X_POSITION_TOLERANCE = 0.006
    X_ORIENTATION_TOLERANCE = 0.075

    STATE_LABELS = {
        'IDLE': '空闲',
        'WAIT_TARGET': '等待并获取按钮位姿',
        'COARSE_APPROACH': 'MoveIt 粗定位',
        'X_ADVANCE': 'MoveIt 按压移动',
        'DONE': '任务完成',
        'ABORT': '任务中止',
    }

    def __init__(self):
        """Create interfaces and validate safety parameters."""
        super().__init__('piper_pbvs_controller')
        self.callback_group = ReentrantCallbackGroup()
        self._declare_parameters()
        self._read_parameters()

        self.data_lock = threading.Lock()
        self.active_lock = threading.Lock()
        self.task_active = False
        self.target_samples = deque(maxlen=self.stable_sample_count)
        self.latest_target = None
        self.latest_target_received = 0.0
        self.latest_tcp = None
        self.latest_tcp_received = 0.0
        self.latest_joint_positions = None
        self.latest_joint_received = 0.0
        self.latest_arm_status = None
        self.active_move_goal = None
        self.last_moveit_arm_target = None
        self.current_state = 'IDLE'

        self.target_sub = self.create_subscription(
            PoseStamped,
            '/piper_vision/button_pose',
            self._target_callback,
            10,
            callback_group=self.callback_group,
        )
        self.tcp_sub = self.create_subscription(
            PoseStamped,
            '/tcp_pose',
            self._tcp_callback,
            10,
            callback_group=self.callback_group,
        )
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
            callback_group=self.callback_group,
        )
        self.status_sub = self.create_subscription(
            PiperStatusMsg,
            '/arm_status',
            self._status_callback,
            10,
            callback_group=self.callback_group,
        )
        self.state_pub = self.create_publisher(String, '/pbvs/state', 10)
        self.desired_tcp_pub = self.create_publisher(
            PoseStamped,
            '/pbvs/desired_tcp_pose',
            10,
        )
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action',
            callback_group=self.callback_group,
        )
        self.interest_client = self.create_client(
            SetInterest,
            '/set_interest',
            callback_group=self.callback_group,
        )
        self.scene_client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene',
            callback_group=self.callback_group,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        self.action_server = ActionServer(
            self,
            PressButton,
            '/press_button',
            execute_callback=self._execute_press,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self._set_state('IDLE')
        self.get_logger().info(
            'Piper MoveIt coarse-positioning controller ready; '
            f'enable_motion={self.enable_motion}, '
            f'orientation_mode={self.orientation_mode}, '
            f'distance_mm={self.distance_m * 1000.0:+.3f}'
        )

    def _declare_parameters(self):
        """Declare motion, convergence, timeout, and collision parameters."""
        defaults = {
            'enable_motion': False,
            'base_frame': 'base_link',
            'tcp_frame': 'tcp_link',
            'flange_frame': 'link6',
            'camera_link_frame': 'camera_link',
            'move_group_name': 'arm',
            'orientation_mode': 'preserve_current_roll',
            'coarse_standoff': 0.08,
            'coarse_horizontal_offset': 0.0,
            'coarse_lateral_error_min': 0.025,
            'coarse_lateral_error_max': 0.035,
            'coarse_axial_tolerance': 0.01,
            'coarse_correction_attempts': 3,
            'distance_mm': 0.0,
            'x_advance_axis_mode': 'base_x',
            'stable_sample_count': 3,
            'stable_position_spread': 0.003,
            'stable_angle_spread': math.radians(3.0),
            'target_acquire_timeout': 5.0,
            'target_pause_age': 0.5,
            'tcp_feedback_timeout': 0.5,
            'moveit_timeout': 20.0,
            'moveit_position_tolerance': 0.002,
            'moveit_orientation_tolerance': 0.05,
            'panel_width': 0.6,
            'panel_height': 1.2,
            'panel_thickness': 0.02,
            'camera_collision_enabled': True,
            'camera_size_x': 0.10,
            'camera_size_y': 0.04,
            'camera_size_z': 0.04,
        }
        self.controller_parameter_names = tuple(defaults)
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        """Read parameters and reject unsafe combinations."""
        values = {
            name: self.get_parameter(name).value
            for name in self.controller_parameter_names
        }
        self.enable_motion = bool(values['enable_motion'])

        string_names = (
            'base_frame',
            'tcp_frame',
            'flange_frame',
            'camera_link_frame',
            'move_group_name',
            'orientation_mode',
            'x_advance_axis_mode',
        )
        for name in string_names:
            value = str(values[name]).strip()
            if not value:
                raise ValueError(f'{name} cannot be empty')
            setattr(self, name, value)
        if self.orientation_mode not in (
            'preserve_current_roll',
            'world_up',
        ):
            raise ValueError(
                'orientation_mode must be preserve_current_roll or world_up'
            )
        if self.x_advance_axis_mode not in ('base_x', 'panel_normal'):
            raise ValueError(
                'x_advance_axis_mode must be base_x or panel_normal'
            )

        integer_names = ('stable_sample_count',)
        for name in integer_names:
            value = int(values[name])
            if value < 1:
                raise ValueError(f'{name} must be positive')
            setattr(self, name, value)

        self.coarse_correction_attempts = int(
            values['coarse_correction_attempts']
        )
        if self.coarse_correction_attempts < 0:
            raise ValueError('coarse_correction_attempts cannot be negative')

        self.distance_m = x_distance_metres(values['distance_mm'])

        bool_names = ('camera_collision_enabled',)
        for name in bool_names:
            setattr(self, name, bool(values[name]))

        signed_float_names = ('coarse_horizontal_offset',)
        for name in signed_float_names:
            value = float(values[name])
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
            setattr(self, name, value)
        if abs(self.coarse_horizontal_offset) > 0.05:
            raise ValueError(
                'coarse_horizontal_offset must be within +/-0.05 m'
            )

        excluded = set(string_names + integer_names + bool_names) | {
            'enable_motion',
            'coarse_correction_attempts',
            'distance_mm',
        } | set(signed_float_names)
        for name, value in values.items():
            if name in excluded:
                continue
            value = float(value)
            if value <= 0.0:
                raise ValueError(f'{name} must be positive')
            setattr(self, name, value)

        if (
            self.coarse_lateral_error_min
            >= self.coarse_lateral_error_max
        ):
            raise ValueError(
                'coarse_lateral_error_min must be smaller than '
                'coarse_lateral_error_max'
            )

    @staticmethod
    def _pose_arrays(pose):
        """Extract numpy position and quaternion arrays from a ROS Pose."""
        return (
            np.array([
                pose.position.x,
                pose.position.y,
                pose.position.z,
            ], dtype=np.float64),
            np.array([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ], dtype=np.float64),
        )

    @staticmethod
    def _format_xyz(position):
        """Format a Cartesian position or displacement for diagnostics."""
        values = np.asarray(position, dtype=np.float64).reshape(3)
        return (
            f'({values[0]:+.4f}, {values[1]:+.4f}, '
            f'{values[2]:+.4f}) m'
        )

    @staticmethod
    def _format_quaternion(quaternion):
        """Format an XYZW quaternion for diagnostics."""
        values = np.asarray(quaternion, dtype=np.float64).reshape(4)
        return (
            f'({values[0]:+.5f}, {values[1]:+.5f}, '
            f'{values[2]:+.5f}, {values[3]:+.5f})'
        )

    @classmethod
    def _format_arm_joints(cls, positions):
        """Format six named arm joint positions in radians."""
        values = np.asarray(positions, dtype=np.float64).reshape(6)
        return ', '.join(
            f'{name}={value:+.6f}'
            for name, value in zip(cls.ARM_JOINT_NAMES, values)
        )

    @classmethod
    def _final_moveit_arm_target(cls, result):
        """Extract the final six-axis target from a MoveGroup result."""
        for field_name in ('executed_trajectory', 'planned_trajectory'):
            robot_trajectory = getattr(result, field_name, None)
            trajectory = getattr(
                robot_trajectory,
                'joint_trajectory',
                None,
            )
            if trajectory is None or not trajectory.points:
                continue
            positions = trajectory.points[-1].positions
            if len(trajectory.joint_names) != len(positions):
                continue
            by_name = dict(zip(trajectory.joint_names, positions))
            if not all(name in by_name for name in cls.ARM_JOINT_NAMES):
                continue
            arm_positions = tuple(
                float(by_name[name]) for name in cls.ARM_JOINT_NAMES
            )
            if all(math.isfinite(value) for value in arm_positions):
                return arm_positions
        return None

    def _log_position_delta(self, label, end_position, start_position):
        """Log a Cartesian displacement vector and its Euclidean norm."""
        delta = np.asarray(end_position) - np.asarray(start_position)
        self.get_logger().info(
            f'【定位诊断】{label}=' + self._format_xyz(delta)
            + f', |{label}|={np.linalg.norm(delta):.4f} m'
        )

    def _pose_message(self, position, quaternion, stamp=None):
        """Construct a base-frame PoseStamped."""
        message = PoseStamped()
        message.header.frame_id = self.base_frame
        message.header.stamp = stamp or self.get_clock().now().to_msg()
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        message.pose.orientation.x = float(quaternion[0])
        message.pose.orientation.y = float(quaternion[1])
        message.pose.orientation.z = float(quaternion[2])
        message.pose.orientation.w = float(quaternion[3])
        return message

    def _target_callback(self, message):
        """Cache valid base-frame button poses for stability checks."""
        if message.header.frame_id != self.base_frame:
            self.get_logger().warn(
                f"Ignoring button pose in '{message.header.frame_id}'"
            )
            return
        position, quaternion = self._pose_arrays(message.pose)
        values = np.concatenate((position, quaternion))
        if not np.all(np.isfinite(values)) or np.linalg.norm(
            quaternion
        ) < 1e-9:
            return
        received = time.monotonic()
        with self.data_lock:
            self.latest_target = copy.deepcopy(message)
            self.latest_target_received = received
            self.target_samples.append(
                (position, quaternion, received)
            )

    def _tcp_callback(self, message):
        """Cache measured Piper TCP feedback."""
        if message.header.frame_id != self.base_frame:
            return
        with self.data_lock:
            self.latest_tcp = copy.deepcopy(message)
            self.latest_tcp_received = time.monotonic()

    def _joint_state_callback(self, message):
        """Cache the latest complete six-axis real joint feedback."""
        if len(message.name) != len(message.position):
            return
        by_name = dict(zip(message.name, message.position))
        if not all(name in by_name for name in self.ARM_JOINT_NAMES):
            return
        positions = tuple(
            float(by_name[name]) for name in self.ARM_JOINT_NAMES
        )
        if not all(math.isfinite(value) for value in positions):
            return
        with self.data_lock:
            self.latest_joint_positions = positions
            self.latest_joint_received = time.monotonic()

    def _status_callback(self, message):
        """Cache Piper hardware status."""
        with self.data_lock:
            self.latest_arm_status = copy.deepcopy(message)

    def _goal_callback(self, goal_request):
        """Accept one non-empty target task at a time."""
        if not goal_request.target_name.strip():
            return GoalResponse.REJECT
        with self.active_lock:
            if self.task_active:
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle):
        """Accept task cancellation; execution performs guarded cleanup."""
        return CancelResponse.ACCEPT

    def _set_state(self, state):
        """Publish the machine-readable state and log its Chinese label."""
        self.current_state = state
        message = String()
        message.data = state
        self.state_pub.publish(message)
        label = self.STATE_LABELS.get(state, state)
        if state in ('IDLE', 'DONE', 'ABORT'):
            self.get_logger().info(
                f'【粗定位状态】{label}（{state}）'
            )
        else:
            self.get_logger().info(
                f'【阶段开始】{label}（{state}）'
            )

    def _stage_success(self, state, detail=''):
        """Log a Chinese success message for one task stage."""
        label = self.STATE_LABELS.get(state, state)
        suffix = f'：{detail}' if detail else ''
        self.get_logger().info(
            f'【阶段成功】{label}（{state}）{suffix}'
        )

    def _stage_failure(self, state, error):
        """Log a Chinese failure message for one task stage."""
        label = self.STATE_LABELS.get(state, state)
        self.get_logger().error(
            f'【阶段失败】{label}（{state}）：{error}'
        )

    def _feedback(self, goal_handle, position_error=0.0,
                  angular_error=0.0):
        """Publish Action feedback for the current state and errors."""
        feedback = PressButton.Feedback()
        feedback.state = self.current_state
        feedback.position_error_m = float(position_error)
        feedback.angular_error_rad = float(angular_error)
        with self.data_lock:
            received = self.latest_target_received
        feedback.target_age_s = float(
            max(0.0, time.monotonic() - received)
            if received else math.inf
        )
        goal_handle.publish_feedback(feedback)

    def _arm_fault(self):
        """Return a hardware fault description, or an empty string."""
        with self.data_lock:
            status = copy.deepcopy(self.latest_arm_status)
        if status is None:
            return ''
        if status.err_code != 0:
            return f'Piper error code {status.err_code}'
        limit_fields = [
            status.joint_1_angle_limit,
            status.joint_2_angle_limit,
            status.joint_3_angle_limit,
            status.joint_4_angle_limit,
            status.joint_5_angle_limit,
            status.joint_6_angle_limit,
        ]
        if any(limit_fields):
            return 'Piper joint angle limit fault'
        communication_fields = [
            status.communication_status_joint_1,
            status.communication_status_joint_2,
            status.communication_status_joint_3,
            status.communication_status_joint_4,
            status.communication_status_joint_5,
            status.communication_status_joint_6,
        ]
        if any(communication_fields):
            return 'Piper joint communication fault'
        return ''

    def _guard(self, goal_handle, ignore_cancel=False):
        """Raise when cancellation or a hardware fault prevents motion."""
        if goal_handle.is_cancel_requested and not ignore_cancel:
            raise TaskCanceled('task canceled')
        fault = self._arm_fault()
        if fault:
            raise TaskFailure(fault)

    @staticmethod
    def _wait_future(future, timeout, goal_handle=None):
        """Wait for a ROS future while subscription callbacks continue."""
        deadline = time.monotonic() + timeout
        while not future.done():
            if (
                goal_handle is not None
                and goal_handle.is_cancel_requested
            ):
                return False
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.02)
        return True

    def _select_interest(self, target_name, goal_handle):
        """Request the unique YOLO class selected by the Action goal."""
        if not self.interest_client.wait_for_service(timeout_sec=3.0):
            raise TaskFailure('/set_interest service unavailable')
        request = SetInterest.Request()
        request.name = target_name
        future = self.interest_client.call_async(request)
        if not self._wait_future(future, 3.0, goal_handle):
            raise TaskFailure('timed out setting YOLO interest')
        response = future.result()
        if response is None or not response.result.startswith(
            'interest changed'
        ):
            detail = response.result if response is not None else 'no response'
            raise TaskFailure(f'YOLO rejected target class: {detail}')

    def _clear_target_tracking(self):
        """Discard target poses captured from an earlier camera viewpoint."""
        with self.data_lock:
            self.target_samples.clear()
            self.latest_target = None
            self.latest_target_received = 0.0

    def _wait_for_stable_target(
        self,
        goal_handle,
        timeout=None,
        timeout_message='stable button pose acquisition timed out',
    ):
        """Wait for enough recent, mutually consistent button poses."""
        timeout = self.target_acquire_timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._guard(goal_handle)
            now = time.monotonic()
            with self.data_lock:
                samples = [
                    sample for sample in self.target_samples
                    if now - sample[2] <= self.target_pause_age
                ]
            if len(samples) >= self.stable_sample_count:
                recent = samples[-self.stable_sample_count:]
                result = average_stable_poses(
                    [sample[0] for sample in recent],
                    [sample[1] for sample in recent],
                    self.stable_position_spread,
                    self.stable_angle_spread,
                )
                if result is not None:
                    return result
            self._feedback(goal_handle)
            time.sleep(0.05)
        raise TaskFailure(timeout_message)

    def _apply_collision_scene(self, button_position, button_quaternion):
        """Add the elevator panel and eye-in-hand camera collision boxes."""
        if not self.scene_client.wait_for_service(timeout_sec=3.0):
            raise TaskFailure('/apply_planning_scene service unavailable')

        scene = PlanningScene()
        scene.is_diff = True
        panel = CollisionObject()
        panel.header.frame_id = self.base_frame
        panel.id = 'pbvs_elevator_panel'
        panel.operation = CollisionObject.ADD
        panel_shape = SolidPrimitive()
        panel_shape.type = SolidPrimitive.BOX
        panel_shape.dimensions = [
            self.panel_width,
            self.panel_height,
            self.panel_thickness,
        ]
        panel_pose = Pose()
        panel_center = offset_along_press_axis(
            button_position,
            button_quaternion,
            self.panel_thickness * 0.5,
        )
        panel_pose.position.x = float(panel_center[0])
        panel_pose.position.y = float(panel_center[1])
        panel_pose.position.z = float(panel_center[2])
        panel_pose.orientation.x = float(button_quaternion[0])
        panel_pose.orientation.y = float(button_quaternion[1])
        panel_pose.orientation.z = float(button_quaternion[2])
        panel_pose.orientation.w = float(button_quaternion[3])
        panel.primitives.append(panel_shape)
        panel.primitive_poses.append(panel_pose)
        scene.world.collision_objects.append(panel)

        if self.camera_collision_enabled:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.flange_frame,
                    self.camera_link_frame,
                    Time(),
                    timeout=Duration(seconds=1.0),
                )
            except TransformException as error:
                raise TaskFailure(
                    f'camera collision TF unavailable: {error}'
                ) from error
            attached = AttachedCollisionObject()
            attached.link_name = self.flange_frame
            attached.touch_links = [
                self.flange_frame,
                'gripper_base',
                self.tcp_frame,
            ]
            attached.object.header.frame_id = self.flange_frame
            attached.object.id = 'pbvs_eye_in_hand_camera'
            attached.object.operation = CollisionObject.ADD
            camera_shape = SolidPrimitive()
            camera_shape.type = SolidPrimitive.BOX
            camera_shape.dimensions = [
                self.camera_size_x,
                self.camera_size_y,
                self.camera_size_z,
            ]
            camera_pose = Pose()
            camera_pose.position.x = transform.transform.translation.x
            camera_pose.position.y = transform.transform.translation.y
            camera_pose.position.z = transform.transform.translation.z
            camera_pose.orientation = transform.transform.rotation
            attached.object.primitives.append(camera_shape)
            attached.object.primitive_poses.append(camera_pose)
            scene.robot_state.is_diff = True
            scene.robot_state.attached_collision_objects.append(attached)

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.scene_client.call_async(request)
        if not self._wait_future(future, 3.0):
            raise TaskFailure('planning scene update timed out')
        if future.result() is None or not future.result().success:
            raise TaskFailure('MoveIt rejected coarse collision scene')

    def _moveit_goal(self, target_pose, plan_only):
        """Construct the coarse MoveGroup goal."""
        constraints = Constraints()
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_frame
        position_constraint.link_name = self.tcp_frame
        region = SolidPrimitive()
        region.type = SolidPrimitive.BOX
        diameter = 2.0 * self.moveit_position_tolerance
        region.dimensions = [diameter, diameter, diameter]
        position_constraint.constraint_region.primitives.append(region)
        position_constraint.constraint_region.primitive_poses.append(
            target_pose.pose
        )
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.base_frame
        orientation_constraint.link_name = self.tcp_frame
        orientation_constraint.orientation = target_pose.pose.orientation
        tolerance = self.moveit_orientation_tolerance
        orientation_constraint.absolute_x_axis_tolerance = tolerance
        orientation_constraint.absolute_y_axis_tolerance = tolerance
        orientation_constraint.absolute_z_axis_tolerance = tolerance
        orientation_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = self.move_group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = plan_only
        goal.planning_options.look_around = False
        goal.planning_options.replan = not plan_only
        goal.planning_options.replan_delay = 1.0
        return goal

    def _run_moveit(
        self,
        target_pose,
        goal_handle,
        stage='coarse approach',
    ):
        """Plan or execute one guarded MoveIt target pose."""
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            raise TaskFailure('/move_action unavailable')
        send_future = self.move_group_client.send_goal_async(
            self._moveit_goal(target_pose, not self.enable_motion)
        )
        if not self._wait_future(send_future, 5.0, goal_handle):
            raise TaskFailure('MoveIt goal submission timed out')
        move_goal = send_future.result()
        if move_goal is None or not move_goal.accepted:
            raise TaskFailure(f'MoveIt rejected {stage}')
        self.active_move_goal = move_goal
        result_future = move_goal.get_result_async()
        deadline = time.monotonic() + self.moveit_timeout
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                move_goal.cancel_goal_async()
                raise TaskCanceled(f'canceled during MoveIt {stage}')
            if time.monotonic() >= deadline:
                move_goal.cancel_goal_async()
                raise TaskFailure(f'MoveIt {stage} timed out')
            self._feedback(goal_handle)
            time.sleep(0.05)
        self.active_move_goal = None
        wrapped_result = result_future.result()
        if (
            wrapped_result is None
            or wrapped_result.result.error_code.val != 1
        ):
            code = (
                wrapped_result.result.error_code.val
                if wrapped_result is not None else 'unknown'
            )
            raise TaskFailure(
                f'MoveIt {stage} failed with error code {code}'
            )
        self.last_moveit_arm_target = self._final_moveit_arm_target(
            wrapped_result.result,
        )

    def _verify_target_pose(
        self,
        goal_handle,
        target_position,
        target_quaternion,
        movement_label,
    ):
        """Require fresh measured TCP feedback at a generic MoveIt target."""
        deadline = time.monotonic() + 3.0
        last_position_error = math.inf
        last_angular_error = math.inf
        while time.monotonic() < deadline:
            self._guard(goal_handle)
            try:
                current_position, current_quaternion = (
                    self._latest_tcp_arrays()
                )
            except TaskFailure:
                time.sleep(0.05)
                continue
            _, _, position_error, angular_error = pose_error(
                target_position,
                target_quaternion,
                current_position,
                current_quaternion,
            )
            last_position_error = position_error
            last_angular_error = angular_error
            self._feedback(goal_handle, position_error, angular_error)
            if (
                position_error <= self.X_POSITION_TOLERANCE
                and angular_error <= self.X_ORIENTATION_TOLERANCE
            ):
                return current_position, current_quaternion
            time.sleep(0.05)
        raise TaskFailure(
            f'{movement_label} did not reach its measured target; '
            f'position_error={last_position_error * 1000.0:.2f} mm, '
            f'angular_error={last_angular_error:.6f} rad'
        )

    def _run_x_advance(
        self,
        goal_handle,
        measured_position,
        measured_quaternion,
        press_quaternion,
    ):
        """Move from measured coarse T0 along the configured advance axis."""
        self._set_state('X_ADVANCE')
        if self.x_advance_axis_mode == 'panel_normal':
            target_position = offset_along_press_axis(
                measured_position,
                press_quaternion,
                self.distance_m,
            )
            axis_label = '视觉锁定的面板按压轴'
            movement_label = 'panel-normal movement'
        else:
            target_position = translated_base_x(
                measured_position,
                self.distance_m,
            )
            axis_label = 'base_link X'
            movement_label = 'base-link X movement'
        target_pose = self._pose_message(
            target_position,
            measured_quaternion,
        )
        self.desired_tcp_pub.publish(target_pose)
        self.get_logger().info(
            f'【按压移动】模式={self.x_advance_axis_mode}，'
            f'从粗定位实测 T0 沿 {axis_label} '
            f'移动 {self.distance_m * 1000.0:+.3f} mm，目标='
            + self._format_xyz(target_position)
        )
        self._run_moveit(
            target_pose,
            goal_handle,
            stage=movement_label,
        )
        reached_position, reached_quaternion = self._verify_target_pose(
            goal_handle,
            target_position,
            measured_quaternion,
            movement_label,
        )
        _, _, position_error, angular_error = pose_error(
            target_position,
            measured_quaternion,
            reached_position,
            reached_quaternion,
        )
        self._stage_success(
            'X_ADVANCE',
            f'{axis_label}移动完成，位置误差='
            f'{position_error * 1000.0:.2f} mm，'
            f'姿态误差={angular_error:.6f} rad',
        )

    def _latest_tcp_arrays(self):
        """Return fresh measured TCP pose arrays."""
        with self.data_lock:
            message = copy.deepcopy(self.latest_tcp)
            received = self.latest_tcp_received
        if message is None:
            raise TaskFailure('no /tcp_pose feedback')
        if time.monotonic() - received > self.tcp_feedback_timeout:
            raise TaskFailure('/tcp_pose feedback is stale')
        return self._pose_arrays(message.pose)

    def _log_coarse_verification_failure(
        self,
        attempt_number,
        total_attempts,
        target_position,
        target_quaternion,
        measured_position,
        measured_quaternion,
        position_error,
        angular_error,
        axial_distance,
        axial_error,
        lateral_vector,
        lateral_error,
    ):
        """Log the last coarse-pose sample without changing task behavior."""
        prefix = (
            '【粗定位验收超差诊断 '
            f'{attempt_number}/{total_attempts}】'
        )
        self.get_logger().error(
            prefix + 'C0目标TCP position='
            + self._format_xyz(target_position)
            + ' quaternion='
            + self._format_quaternion(target_quaternion)
        )
        if measured_position is None or measured_quaternion is None:
            self.get_logger().error(prefix + 'T0实测TCP=unavailable')
            self.get_logger().error(prefix + 'T0-C0=unavailable')
            self.get_logger().error(prefix + '位置误差=unavailable')
            self.get_logger().error(prefix + '姿态误差=unavailable')
            self.get_logger().error(prefix + '法向/横向误差=unavailable')
        else:
            delta = np.asarray(measured_position) - np.asarray(
                target_position
            )
            self.get_logger().error(
                prefix + 'T0实测TCP position='
                + self._format_xyz(measured_position)
                + ' quaternion='
                + self._format_quaternion(measured_quaternion)
            )
            self.get_logger().error(
                prefix + 'T0-C0=' + self._format_xyz(delta)
            )
            self.get_logger().error(
                prefix
                + f'位置误差={position_error:.6f} m '
                + f'({position_error * 1000.0:.2f} mm)'
            )
            self.get_logger().error(
                prefix
                + f'姿态误差={angular_error:.6f} rad '
                + f'({math.degrees(angular_error):.2f} deg)'
            )
            self.get_logger().error(
                prefix
                + f'法向距离={axial_distance:.6f} m, '
                + f'法向距离误差={axial_error:+.6f} m, '
                + f'允许±{self.coarse_axial_tolerance:.4f} m'
            )
            self.get_logger().error(
                prefix + '横向误差向量='
                + self._format_xyz(lateral_vector)
                + f', 模长={lateral_error:.6f} m, '
                + '允许范围=['
                + f'{self.coarse_lateral_error_min:.4f}, '
                + f'{self.coarse_lateral_error_max:.4f}] m'
            )

        target_joints = self.last_moveit_arm_target
        if target_joints is None:
            self.get_logger().error(prefix + '目标关节角(rad)=unavailable')
        else:
            self.get_logger().error(
                prefix + '目标关节角(rad): '
                + self._format_arm_joints(target_joints)
            )

        with self.data_lock:
            measured_joints = self.latest_joint_positions
            joint_received = self.latest_joint_received
        if measured_joints is None or joint_received <= 0.0:
            self.get_logger().error(prefix + '实测关节角(rad)=unavailable')
        else:
            joint_age = max(0.0, time.monotonic() - joint_received)
            freshness = (
                'fresh'
                if joint_age <= self.tcp_feedback_timeout else 'stale'
            )
            self.get_logger().error(
                prefix + '实测关节角(rad): '
                + self._format_arm_joints(measured_joints)
                + f', age={joint_age:.3f} s ({freshness})'
            )

    def _verify_coarse_pose(
        self,
        goal_handle,
        button_position,
        position,
        quaternion,
        attempt_number,
        total_attempts,
    ):
        """Verify measured TCP feedback after MoveIt reports success."""
        deadline = time.monotonic() + 3.0
        last_position = None
        last_quaternion = None
        last_position_error = math.inf
        last_angular_error = math.inf
        last_axial_distance = math.nan
        last_axial_error = math.nan
        last_lateral_vector = None
        last_lateral_error = math.inf
        while time.monotonic() < deadline:
            self._guard(goal_handle)
            try:
                current_position, current_quaternion = (
                    self._latest_tcp_arrays()
                )
            except TaskFailure:
                time.sleep(0.05)
                continue
            _, _, position_error, angular_error = pose_error(
                position,
                quaternion,
                current_position,
                current_quaternion,
            )
            last_position = current_position
            last_quaternion = current_quaternion
            last_position_error = position_error
            last_angular_error = angular_error
            (
                axial_distance,
                axial_error,
                lateral_vector,
                lateral_error,
            ) = coarse_standoff_errors(
                button_position,
                current_position,
                quaternion,
                self.coarse_standoff,
            )
            last_axial_distance = axial_distance
            last_axial_error = axial_error
            last_lateral_vector = lateral_vector
            last_lateral_error = lateral_error
            self._feedback(
                goal_handle,
                position_error,
                angular_error,
            )
            if coarse_pose_is_acceptable(
                axial_error,
                lateral_error,
                angular_error,
                self.coarse_axial_tolerance,
                self.coarse_lateral_error_min,
                self.coarse_lateral_error_max,
                1.5 * self.moveit_orientation_tolerance,
            ):
                # MoveIt accepts a region around the requested pose. Return
                # the pose actually reached for final diagnostics.
                return current_position, current_quaternion
            time.sleep(0.05)
        self._log_coarse_verification_failure(
            attempt_number,
            total_attempts,
            position,
            quaternion,
            last_position,
            last_quaternion,
            last_position_error,
            last_angular_error,
            last_axial_distance,
            last_axial_error,
            last_lateral_vector,
            last_lateral_error,
        )
        return None

    def _control_quaternion(
        self,
        detected_quaternion,
        roll_reference_quaternion,
    ):
        """Build a control orientation from the detected press axis."""
        if self.orientation_mode == 'world_up':
            return detected_quaternion
        press_axis = quaternion_to_matrix(detected_quaternion)[:, 2]
        return align_tool_z_preserve_roll(
            roll_reference_quaternion,
            press_axis,
        )

    def _result(self, success, message):
        """Construct a PressButton result."""
        result = PressButton.Result()
        result.success = success
        result.message = message
        return result

    def _execute_press(self, goal_handle):
        """Acquire one button target and complete MoveIt coarse positioning."""
        with self.active_lock:
            if self.task_active:
                goal_handle.abort()
                return self._result(False, 'another press task is active')
            self.task_active = True

        try:
            self.last_moveit_arm_target = None
            target_name = goal_handle.request.target_name.strip()
            self._set_state('WAIT_TARGET')
            self._clear_target_tracking()
            self._select_interest(target_name, goal_handle)
            button_position, button_quaternion = (
                self._wait_for_stable_target(goal_handle)
            )
            self._stage_success(
                'WAIT_TARGET',
                f"已取得目标 '{target_name}' 的稳定按钮位姿",
            )
            roll_reference_quaternion = None
            if self.orientation_mode == 'preserve_current_roll':
                _, roll_reference_quaternion = self._latest_tcp_arrays()
            control_quaternion = self._control_quaternion(
                button_quaternion,
                roll_reference_quaternion,
            )
            uncompensated_coarse_position = offset_along_press_axis(
                button_position,
                control_quaternion,
                -self.coarse_standoff,
            )
            coarse_position = offset_along_panel_horizontal(
                uncompensated_coarse_position,
                button_quaternion,
                self.coarse_horizontal_offset,
            )
            coarse_quaternion = control_quaternion
            self.get_logger().info(
                '【粗定位水平补偿】'
                f'{self.coarse_horizontal_offset * 1000.0:+.1f} mm '
                '（正值向左，负值向右）'
            )
            coarse_pose = self._pose_message(
                coarse_position,
                coarse_quaternion,
            )
            self.desired_tcp_pub.publish(coarse_pose)
            self._apply_collision_scene(
                button_position,
                button_quaternion,
            )
            self.get_logger().info(
                '【阶段成功】MoveIt 碰撞场景：电梯面板和相机模型已更新'
            )

            self._set_state('COARSE_APPROACH')
            total_coarse_attempts = coarse_total_attempts(
                self.enable_motion,
                self.coarse_correction_attempts,
            )
            verified_pose = None
            successful_attempt = 0
            for attempt_index in range(total_coarse_attempts):
                attempt_number = attempt_index + 1
                if attempt_number > 1:
                    self.get_logger().warning(
                        '【粗定位校正】上一次实测超差，继续使用同一 '
                        f'C0 执行第 {attempt_number}/{total_coarse_attempts} '
                        '次 MoveIt'
                    )
                self._run_moveit(coarse_pose, goal_handle)
                if not self.enable_motion:
                    self._stage_success(
                        'COARSE_APPROACH',
                        'MoveIt 规划成功；当前为 dry-run，'
                        '未发送运动命令',
                    )
                    self._set_state('DONE')
                    if self.distance_m != 0.0:
                        self.get_logger().warning(
                            '已配置 distance_mm='
                            f'{self.distance_m * 1000.0:+.3f}，但当前为 '
                            'dry-run，跳过按压轴实机移动'
                        )
                    goal_handle.succeed()
                    return self._result(
                        True,
                        'MoveIt 粗定位规划成功；'
                        'dry-run 未发送运动命令',
                    )
                verified_pose = self._verify_coarse_pose(
                    goal_handle,
                    button_position,
                    coarse_position,
                    coarse_quaternion,
                    attempt_number,
                    total_coarse_attempts,
                )
                if verified_pose is not None:
                    successful_attempt = attempt_number
                    break

            if verified_pose is None:
                raise TaskFailure(
                    'measured TCP failed button-frame coarse alignment '
                    'after MoveIt correction'
                )

            measured_position, _ = verified_pose
            (
                axial_distance,
                axial_error,
                lateral_vector,
                lateral_error,
            ) = coarse_standoff_errors(
                button_position,
                measured_position,
                coarse_quaternion,
                self.coarse_standoff,
            )
            self._stage_success(
                'COARSE_APPROACH',
                'MoveIt 返回成功，实测 TCP 已进入粗定位验收范围'
                f'（第 {successful_attempt}/{total_coarse_attempts} 次）',
            )
            self.get_logger().info(
                '【定位诊断】首次按钮位置 B0='
                + self._format_xyz(button_position)
            )
            self.get_logger().info(
                '【定位诊断】未补偿粗定位目标 C0_raw='
                + self._format_xyz(uncompensated_coarse_position)
            )
            self.get_logger().info(
                '【定位诊断】补偿后 MoveIt 粗定位目标 C0='
                + self._format_xyz(coarse_position)
            )
            self._log_position_delta(
                '粗定位目标补偿 C0-C0_raw',
                coarse_position,
                uncompensated_coarse_position,
            )
            self.get_logger().info(
                '【定位诊断】真机到达位置 T0='
                + self._format_xyz(measured_position)
            )
            self._log_position_delta(
                'MoveIt执行误差 T0-C0',
                measured_position,
                coarse_position,
            )
            horizontal_axis = quaternion_to_matrix(button_quaternion)[:, 0]
            measured_horizontal_offset = float(np.dot(
                measured_position - uncompensated_coarse_position,
                horizontal_axis,
            ))
            self.get_logger().info(
                '【定位诊断】实测相对未补偿目标水平位移='
                f'{measured_horizontal_offset * 1000.0:+.2f} mm '
                '（正值向左，负值向右）'
            )
            self.get_logger().info(
                '【定位诊断】B0-T0法向距离='
                f'{axial_distance:.6f} m, '
                f'法向误差={axial_error:+.6f} m'
            )
            self.get_logger().info(
                '【定位诊断】B0-T0横向误差向量='
                + self._format_xyz(lateral_vector)
                + f', 模长={lateral_error:.6f} m, '
                + '允许范围=['
                + f'{self.coarse_lateral_error_min:.4f}, '
                + f'{self.coarse_lateral_error_max:.4f}] m'
            )
            if self.distance_m != 0.0:
                measured_position, measured_quaternion = verified_pose
                self._run_x_advance(
                    goal_handle,
                    measured_position,
                    measured_quaternion,
                    coarse_quaternion,
                )
            self._set_state('DONE')
            goal_handle.succeed()
            if self.distance_m != 0.0:
                return self._result(
                    True,
                    'MoveIt 初定位及按压轴移动完成'
                    f'（{self.x_advance_axis_mode}）',
                )
            return self._result(
                True,
                'MoveIt 初定位完成；机械臂保持在实测 T0，未执行 PBVS',
            )

        except TaskCanceled as error:
            failed_state = self.current_state
            self._stage_failure(failed_state, f'任务被取消：{error}')
            self._set_state('ABORT')
            goal_handle.canceled()
            return self._result(False, str(error))
        except Exception as error:
            failed_state = self.current_state
            self._stage_failure(failed_state, error)
            self._set_state('ABORT')
            goal_handle.abort()
            return self._result(False, str(error))
        finally:
            self.active_move_goal = None
            with self.active_lock:
                self.task_active = False
            if self.current_state in ('DONE', 'ABORT'):
                self._set_state('IDLE')

    def destroy_node(self):
        """Destroy the Action server before the ROS node."""
        self.action_server.destroy()
        return super().destroy_node()


def main(args=None):
    """Run the coarse controller with concurrent action and callbacks."""
    rclpy.init(args=args)
    node = PiperPbvsController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
