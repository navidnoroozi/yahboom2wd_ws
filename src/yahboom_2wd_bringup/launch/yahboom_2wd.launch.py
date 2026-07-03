from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    serial_port = LaunchConfiguration('serial_port')
    command_mode = LaunchConfiguration('command_mode')
    config_file = LaunchConfiguration('config_file')
    publish_tf = LaunchConfiguration('publish_tf')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation = LaunchConfiguration('wheel_separation')
    linear_cmd_scale = LaunchConfiguration('linear_cmd_scale')
    angular_cmd_scale = LaunchConfiguration('angular_cmd_scale')

    robot_xacro = PathJoinSubstitution([
        FindPackageShare('yahboom_2wd_description'),
        'urdf',
        'yahboom_2wd.urdf.xacro',
    ])

    robot_description = Command([
        'xacro ', robot_xacro,
        ' robot_name:=', namespace,
        ' wheel_radius:=', wheel_radius,
        ' wheel_separation:=', wheel_separation,
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot1'),
        DeclareLaunchArgument('serial_port', default_value='/dev/myserial'),
        DeclareLaunchArgument('command_mode', default_value='motion'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('wheel_radius', default_value='0.0325'),
        DeclareLaunchArgument('wheel_separation', default_value='0.120'),
        DeclareLaunchArgument('linear_cmd_scale', default_value='1.7'),
        DeclareLaunchArgument('angular_cmd_scale', default_value='1.0'),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('yahboom_2wd_bringup'),
                'config',
                'yahboom_2wd.yaml',
            ]),
        ),
        GroupAction([
            PushRosNamespace(namespace),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_description,
                    'publish_frequency': 30.0,
                }],
            ),
            Node(
                package='yahboom_2wd_driver',
                executable='yahboom_2wd_node',
                name='yahboom_2wd_node',
                output='screen',
                parameters=[
                    config_file,
                    {
                        'serial_port': serial_port,
                        'command_mode': command_mode,
                        'publish_tf': publish_tf,
                        'wheel_radius': wheel_radius,
                        'wheel_separation': wheel_separation,
                        'linear_cmd_scale': linear_cmd_scale,
                        'angular_cmd_scale': angular_cmd_scale,
                        'odom_frame_id': PythonExpression(["'", namespace, "/odom'"]),
                        'base_frame_id': PythonExpression(["'", namespace, "/base_footprint'"]),
                        'imu_frame_id': PythonExpression(["'", namespace, "/imu_link'"]),
                    },
                ],
            ),
        ]),
    ])
