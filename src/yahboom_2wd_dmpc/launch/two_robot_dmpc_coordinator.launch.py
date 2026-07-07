from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot1_controller_endpoint", default_value="tcp://192.168.178.51:5601"),
        DeclareLaunchArgument("robot2_controller_endpoint", default_value="tcp://192.168.178.52:5602"),
        DeclareLaunchArgument("world_frame", default_value="map"),
        DeclareLaunchArgument("robot1_initial_x", default_value="0.0"),
        DeclareLaunchArgument("robot1_initial_y", default_value="-0.45"),
        DeclareLaunchArgument("robot1_initial_yaw", default_value="0.0"),
        DeclareLaunchArgument("robot2_initial_x", default_value="0.0"),
        DeclareLaunchArgument("robot2_initial_y", default_value="0.45"),
        DeclareLaunchArgument("robot2_initial_yaw", default_value="0.0"),
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("rate_hz", default_value="5.0"),
        DeclareLaunchArgument("max_linear_speed", default_value="0.07"),
        DeclareLaunchArgument("max_angular_speed", default_value="0.35"),
        DeclareLaunchArgument("u_bound", default_value="0.08"),
        DeclareLaunchArgument("d_safe", default_value="0.65"),
        DeclareLaunchArgument("formation_margin", default_value="0.15"),
        DeclareLaunchArgument("obstacles_enabled", default_value="false"),
        Node(
            package="yahboom_2wd_dmpc",
            executable="dmpc_coordinator_ros_node",
            name="dmpc_coordinator_ros_node",
            output="screen",
            parameters=[{
                "robot_namespaces": ["robot1", "robot2"],
                "agent_ids": [1, 2],
                # Do not pass a LaunchConfiguration list as a STRING_ARRAY.
                # Humble can concatenate substitutions into one scalar string.
                # Pass scalar endpoint parameters and let the node build the list.
                "robot1_controller_endpoint": LaunchConfiguration("robot1_controller_endpoint"),
                "robot2_controller_endpoint": LaunchConfiguration("robot2_controller_endpoint"),
                "world_frame": LaunchConfiguration("world_frame"),
                "robot1_initial_x": LaunchConfiguration("robot1_initial_x"),
                "robot1_initial_y": LaunchConfiguration("robot1_initial_y"),
                "robot1_initial_yaw": LaunchConfiguration("robot1_initial_yaw"),
                "robot2_initial_x": LaunchConfiguration("robot2_initial_x"),
                "robot2_initial_y": LaunchConfiguration("robot2_initial_y"),
                "robot2_initial_yaw": LaunchConfiguration("robot2_initial_yaw"),
                "rate_hz": LaunchConfiguration("rate_hz"),
                "enable_motion": LaunchConfiguration("enable_motion"),
                "model": "single_integrator",
                "graph": "complete",
                "objective_mode": "safe_formation",
                "auto_M": False,
                "M_manual": 5,
                "u_bound": LaunchConfiguration("u_bound"),
                "d_safe": LaunchConfiguration("d_safe"),
                "formation_margin": LaunchConfiguration("formation_margin"),
                "safety_enabled": True,
                "obstacles_enabled": LaunchConfiguration("obstacles_enabled"),
                "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                "max_angular_speed": LaunchConfiguration("max_angular_speed"),
                "world_command_to_speed_scale": 1.0,
                "heading_gain": 1.8,
                "linear_deadband": 0.005,
                "stop_for_heading_error_rad": 1.20,
                "allow_reverse": False,
            }],
        ),
    ])
