"""Launch YOLO hand-eye perception and MoveIt coarse positioning."""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the elevator button press launch description."""
    vision_launch = PathJoinSubstitution([
        FindPackageShare('piper_vision'),
        'launch',
        'yolo_handeye.launch.py',
    ])
    controller_config = PathJoinSubstitution([
        FindPackageShare('piper_pbvs_control'),
        'config',
        'pbvs_control.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            description='Absolute path to a YOLO11 detect model',
        ),
        DeclareLaunchArgument('conda_env', default_value='yolo11'),
        DeclareLaunchArgument('device', default_value=''),
        DeclareLaunchArgument('bg_removal', default_value='false'),
        DeclareLaunchArgument('enable_handeye_tf', default_value='true'),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('floor_number', default_value='1'),
        DeclareLaunchArgument(
            'home_joint_positions',
            default_value=(
                '[1.613151344, 0.18368532, -0.955564876, '
                '0.10300682, 0.785450988, -0.042511028]'
            ),
            description='Six arm home joint positions in radians',
        ),
        DeclareLaunchArgument(
            'home_joint_tolerance',
            default_value='0.01',
        ),
        DeclareLaunchArgument(
            'home_velocity_scaling_factor',
            default_value='0.01',
        ),
        DeclareLaunchArgument(
            'home_acceleration_scaling_factor',
            default_value='0.01',
        ),
        DeclareLaunchArgument('home_timeout', default_value='20.0'),
        DeclareLaunchArgument('press_timeout', default_value='120.0'),
        DeclareLaunchArgument(
            'moveit_velocity_scaling_factor',
            default_value='0.07',
            description=(
                'MoveIt velocity scaling for coarse and panel-normal moves'
            ),
        ),
        DeclareLaunchArgument(
            'moveit_acceleration_scaling_factor',
            default_value='0.07',
            description=(
                'MoveIt acceleration scaling for coarse and panel-normal moves'
            ),
        ),
        DeclareLaunchArgument(
            'orientation_mode',
            default_value='preserve_current_roll',
            description=(
                'TCP orientation policy: preserve_current_roll or world_up'
            ),
        ),
        DeclareLaunchArgument('coarse_standoff', default_value='0.08'),
        DeclareLaunchArgument(
            'coarse_horizontal_offset',
            default_value='0.0',
            description=(
                'Coarse target horizontal compensation in metres; '
                'positive is left facing the panel'
            ),
        ),
        DeclareLaunchArgument(
            'coarse_lateral_error_min',
            default_value='0.009',
        ),
        DeclareLaunchArgument(
            'coarse_lateral_error_max',
            default_value='0.019',
        ),
        DeclareLaunchArgument(
            'coarse_axial_tolerance',
            default_value='0.01',
        ),
        DeclareLaunchArgument(
            'coarse_correction_attempts',
            default_value='3',
        ),
        DeclareLaunchArgument(
            'distance_mm',
            default_value='0.0',
            description=(
                'Optional post-coarse advance displacement in mm'
            ),
        ),
        DeclareLaunchArgument(
            'x_advance_axis_mode',
            default_value='base_x',
            description='Advance axis: base_x or panel_normal',
        ),
        DeclareLaunchArgument('plane_outer_scale', default_value='2.0'),
        DeclareLaunchArgument('plane_inner_scale', default_value='1.0'),
        DeclareLaunchArgument(
            'plane_ransac_threshold',
            default_value='0.003',
        ),
        DeclareLaunchArgument('plane_min_points', default_value='60'),
        DeclareLaunchArgument(
            'plane_min_inlier_ratio',
            default_value='0.6',
        ),
        DeclareLaunchArgument('plane_max_rms', default_value='0.004'),
        DeclareLaunchArgument(
            'plane_max_depth_deviation',
            default_value='0.03',
        ),
        DeclareLaunchArgument('plane_sample_step', default_value='2'),
        DeclareLaunchArgument('plane_lock_sample_count', default_value='5'),
        DeclareLaunchArgument(
            'plane_lock_max_offset_spread',
            default_value='0.005',
        ),
        DeclareLaunchArgument(
            'plane_lock_max_angle_spread',
            default_value='0.052359878',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vision_launch),
            launch_arguments={
                'model_path': LaunchConfiguration('model_path'),
                'conda_env': LaunchConfiguration('conda_env'),
                'device': LaunchConfiguration('device'),
                'interest': 'all',
                'bg_removal': LaunchConfiguration('bg_removal'),
                'enable_handeye_tf': LaunchConfiguration(
                    'enable_handeye_tf'
                ),
                'calibrated_frame': 'camera_link',
                'plane_outer_scale': LaunchConfiguration(
                    'plane_outer_scale'
                ),
                'plane_inner_scale': LaunchConfiguration(
                    'plane_inner_scale'
                ),
                'plane_ransac_threshold': LaunchConfiguration(
                    'plane_ransac_threshold'
                ),
                'plane_min_points': LaunchConfiguration(
                    'plane_min_points'
                ),
                'plane_min_inlier_ratio': LaunchConfiguration(
                    'plane_min_inlier_ratio'
                ),
                'plane_max_rms': LaunchConfiguration('plane_max_rms'),
                'plane_max_depth_deviation': LaunchConfiguration(
                    'plane_max_depth_deviation'
                ),
                'plane_sample_step': LaunchConfiguration(
                    'plane_sample_step'
                ),
                'plane_lock_sample_count': LaunchConfiguration(
                    'plane_lock_sample_count'
                ),
                'plane_lock_max_offset_spread': LaunchConfiguration(
                    'plane_lock_max_offset_spread'
                ),
                'plane_lock_max_angle_spread': LaunchConfiguration(
                    'plane_lock_max_angle_spread'
                ),
            }.items(),
        ),
        Node(
            package='piper_pbvs_control',
            executable='pbvs_controller',
            name='piper_pbvs_controller',
            output='screen',
            parameters=[
                controller_config,
                {
                    'enable_motion': ParameterValue(
                        LaunchConfiguration('enable_motion'),
                        value_type=bool,
                    ),
                    'orientation_mode': ParameterValue(
                        LaunchConfiguration('orientation_mode'),
                        value_type=str,
                    ),
                    'moveit_velocity_scaling_factor': ParameterValue(
                        LaunchConfiguration(
                            'moveit_velocity_scaling_factor'
                        ),
                        value_type=float,
                    ),
                    'moveit_acceleration_scaling_factor': ParameterValue(
                        LaunchConfiguration(
                            'moveit_acceleration_scaling_factor'
                        ),
                        value_type=float,
                    ),
                    'coarse_standoff': ParameterValue(
                        LaunchConfiguration('coarse_standoff'),
                        value_type=float,
                    ),
                    'coarse_horizontal_offset': ParameterValue(
                        LaunchConfiguration('coarse_horizontal_offset'),
                        value_type=float,
                    ),
                    'coarse_lateral_error_min': ParameterValue(
                        LaunchConfiguration('coarse_lateral_error_min'),
                        value_type=float,
                    ),
                    'coarse_lateral_error_max': ParameterValue(
                        LaunchConfiguration('coarse_lateral_error_max'),
                        value_type=float,
                    ),
                    'coarse_axial_tolerance': ParameterValue(
                        LaunchConfiguration('coarse_axial_tolerance'),
                        value_type=float,
                    ),
                    'coarse_correction_attempts': ParameterValue(
                        LaunchConfiguration('coarse_correction_attempts'),
                        value_type=int,
                    ),
                    'distance_mm': ParameterValue(
                        LaunchConfiguration('distance_mm'),
                        value_type=float,
                    ),
                    'x_advance_axis_mode': ParameterValue(
                        LaunchConfiguration('x_advance_axis_mode'),
                        value_type=str,
                    ),
                },
            ],
        ),
        Node(
            package='piper_pbvs_control',
            executable='elevator_sequence',
            name='elevator_sequence',
            output='screen',
            parameters=[
                controller_config,
                {
                    'enable_motion': ParameterValue(
                        LaunchConfiguration('enable_motion'),
                        value_type=bool,
                    ),
                    'floor_number': ParameterValue(
                        LaunchConfiguration('floor_number'),
                        value_type=int,
                    ),
                    'home_joint_positions': ParameterValue(
                        LaunchConfiguration('home_joint_positions'),
                        value_type=List[float],
                    ),
                    'home_joint_tolerance': ParameterValue(
                        LaunchConfiguration('home_joint_tolerance'),
                        value_type=float,
                    ),
                    'home_velocity_scaling_factor': ParameterValue(
                        LaunchConfiguration(
                            'home_velocity_scaling_factor'
                        ),
                        value_type=float,
                    ),
                    'home_acceleration_scaling_factor': ParameterValue(
                        LaunchConfiguration(
                            'home_acceleration_scaling_factor'
                        ),
                        value_type=float,
                    ),
                    'home_timeout': ParameterValue(
                        LaunchConfiguration('home_timeout'),
                        value_type=float,
                    ),
                    'press_timeout': ParameterValue(
                        LaunchConfiguration('press_timeout'),
                        value_type=float,
                    ),
                },
            ],
        ),
    ])
