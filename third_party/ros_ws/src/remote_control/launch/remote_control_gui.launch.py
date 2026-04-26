from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('remote_control'), 'config', 'xbox_like.yaml'
    ])
    config = LaunchConfiguration('config')
    device_id = LaunchConfiguration('device_id')
    use_joy = LaunchConfiguration('use_joy')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to remote_control parameter YAML file',
        ),
        DeclareLaunchArgument(
            'device_id',
            default_value='0',
            description='Joystick device index for joy_node (used only if a joystick is present)',
        ),
        DeclareLaunchArgument(
            'use_joy',
            default_value='true',
            description='Whether to start joy_node alongside the GUI remote control node',
        ),
        Node(
            package='remote_control',
            executable='remote_control_gui_node',
            name='remote_control_node',
            parameters=[config, {'gui.config_path': config}],
            output='screen',
        ),
        LogInfo(
            condition=IfCondition(use_joy),
            msg='[remote_control_gui] use_joy=true; starting joy_node'
        ),
        LogInfo(
            condition=UnlessCondition(use_joy),
            msg='[remote_control_gui] use_joy=false; GUI keyboard-only mode'
        ),
        Node(
            condition=IfCondition(use_joy),
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': device_id,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
            output='screen',
        ),
    ])
