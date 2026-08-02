"""Launch YOLO hand-eye perception and the guarded PBVS controller."""

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
        DeclareLaunchArgument('enable_press', default_value='false'),
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
            'coarse_lateral_tolerance',
            default_value='0.007',
        ),
        DeclareLaunchArgument(
            'coarse_axial_tolerance',
            default_value='0.01',
        ),
        DeclareLaunchArgument(
            'coarse_correction_attempts',
            default_value='1',
        ),
        DeclareLaunchArgument(
            'target_reacquire_timeout',
            default_value='8.0',
        ),
        DeclareLaunchArgument(
            'reacquire_max_normal_drift',
            default_value='0.02',
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
                    'enable_press': ParameterValue(
                        LaunchConfiguration('enable_press'),
                        value_type=bool,
                    ),
                    'orientation_mode': ParameterValue(
                        LaunchConfiguration('orientation_mode'),
                        value_type=str,
                    ),
                    'coarse_standoff': ParameterValue(
                        LaunchConfiguration('coarse_standoff'),
                        value_type=float,
                    ),
                    'coarse_horizontal_offset': ParameterValue(
                        LaunchConfiguration('coarse_horizontal_offset'),
                        value_type=float,
                    ),
                    'coarse_lateral_tolerance': ParameterValue(
                        LaunchConfiguration('coarse_lateral_tolerance'),
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
                    'target_reacquire_timeout': ParameterValue(
                        LaunchConfiguration('target_reacquire_timeout'),
                        value_type=float,
                    ),
                    'reacquire_max_normal_drift': ParameterValue(
                        LaunchConfiguration('reacquire_max_normal_drift'),
                        value_type=float,
                    ),
                },
            ],
        ),
    ])
