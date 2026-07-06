from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("agent_id", description="1 for robot1, 2 for robot2"),
        DeclareLaunchArgument("bind_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("bind_port", default_value=""),
        DeclareLaunchArgument("n_agents", default_value="2"),
        DeclareLaunchArgument("u_bound", default_value="0.08"),
        DeclareLaunchArgument("d_safe", default_value="0.65"),
        DeclareLaunchArgument("formation_margin", default_value="0.15"),
        DeclareLaunchArgument("obstacles_enabled", default_value="false"),
        Node(
            package="yahboom_2wd_dmpc",
            executable="dmpc_controller_node",
            name=["dmpc_controller_", LaunchConfiguration("agent_id")],
            output="screen",
            arguments=[
                "--agent-id", LaunchConfiguration("agent_id"),
                "--bind-host", LaunchConfiguration("bind_host"),
                "--n-agents", LaunchConfiguration("n_agents"),
                "--model", "single_integrator",
                "--graph", "complete",
                "--auto-M", "false",
                "--M-manual", "5",
                "--u-bound", LaunchConfiguration("u_bound"),
                "--d-safe", LaunchConfiguration("d_safe"),
                "--formation-margin", LaunchConfiguration("formation_margin"),
                "--safety-enabled", "true",
                "--obstacles-enabled", LaunchConfiguration("obstacles_enabled"),
            ],
        ),
    ])
