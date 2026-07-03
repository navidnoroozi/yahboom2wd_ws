from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_namespace = LaunchConfiguration('robot_namespace')
    scenario = LaunchConfiguration('scenario')
    linear_speed = LaunchConfiguration('linear_speed')
    angular_speed = LaunchConfiguration('angular_speed')
    distance = LaunchConfiguration('distance')
    radius = LaunchConfiguration('radius')
    arc_angle = LaunchConfiguration('arc_angle')
    rotation_angle = LaunchConfiguration('rotation_angle')
    turn_direction = LaunchConfiguration('turn_direction')
    amplitude = LaunchConfiguration('amplitude')
    wavelength = LaunchConfiguration('wavelength')
    path_length = LaunchConfiguration('path_length')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')

    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value='robot1'),
        DeclareLaunchArgument('scenario', default_value='straight'),
        DeclareLaunchArgument('linear_speed', default_value='0.08'),
        DeclareLaunchArgument('angular_speed', default_value='0.20'),
        DeclareLaunchArgument('distance', default_value='1.0'),
        DeclareLaunchArgument('radius', default_value='1.0'),
        DeclareLaunchArgument('arc_angle', default_value='1.57079632679'),
        DeclareLaunchArgument('rotation_angle', default_value='1.57079632679'),
        DeclareLaunchArgument('turn_direction', default_value='left'),
        DeclareLaunchArgument('amplitude', default_value='0.20'),
        DeclareLaunchArgument('wavelength', default_value='2.0'),
        DeclareLaunchArgument('path_length', default_value='2.0'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.12'),
        DeclareLaunchArgument('max_angular_speed', default_value='0.60'),
        Node(
            package='yahboom_2wd_tests',
            executable='path_follower_node',
            name='path_follower_node',
            output='screen',
            parameters=[{
                'robot_namespace': robot_namespace,
                'scenario': scenario,
                'linear_speed': linear_speed,
                'angular_speed': angular_speed,
                'distance': distance,
                'radius': radius,
                'arc_angle': arc_angle,
                'rotation_angle': rotation_angle,
                'turn_direction': turn_direction,
                'amplitude': amplitude,
                'wavelength': wavelength,
                'path_length': path_length,
                'max_linear_speed': max_linear_speed,
                'max_angular_speed': max_angular_speed,
            }],
        ),
    ])
