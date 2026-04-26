import glob

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_actions(context):
    config = LaunchConfiguration('config').perform(context)
    try:
        device_id = int(LaunchConfiguration('device_id').perform(context))
    except ValueError:
        device_id = 0

    actions = [
        Node(
            package='remote_control',
            executable='remote_control_node',
            name='remote_control_node',
            parameters=[config],
            output='screen',
        ),
    ]

    js_devices = sorted(glob.glob('/dev/input/js*'))
    if js_devices:
        actions.append(LogInfo(
            msg=f'[remote_control] joystick detected ({js_devices[0]}); starting joy_node'
        ))
        actions.append(Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': device_id,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
            output='screen',
        ))
    else:
        actions.append(LogInfo(
            msg='[remote_control] no /dev/input/js* found; skipping joy_node (keyboard-only mode)'
        ))

    return actions


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('remote_control'), 'config', 'xbox_like.yaml'
    ])

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
        OpaqueFunction(function=_build_actions),
    ])
