from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("rosmaster_a1"))
    default_config = package_share / "config" / "default_route.yaml"
    default_rviz_config = package_share / "rviz" / "simulator.rviz"
    robot_description = (package_share / "urdf" / "simulator_vehicle.urdf").read_text()

    route_config = LaunchConfiguration("route_config")
    enable_metrics = LaunchConfiguration("enable_metrics")
    enable_rviz = LaunchConfiguration("enable_rviz")
    metrics_log_dir = LaunchConfiguration("metrics_log_dir")
    metrics_summary_period_s = LaunchConfiguration("metrics_summary_period_s")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "route_config",
            default_value=str(default_config),
            description="Parameter file for the planner, controller, simulator, and metrics logger.",
        ),
        DeclareLaunchArgument(
            "enable_metrics",
            default_value="true",
            description="Start the metrics logger node.",
        ),
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="false",
            description="Start RViz with the simulator visualization configuration.",
        ),
        DeclareLaunchArgument(
            "metrics_log_dir",
            default_value="metrics_logs",
            description="Directory where the metrics logger writes CSV files.",
        ),
        DeclareLaunchArgument(
            "metrics_summary_period_s",
            default_value="2.0",
            description="Seconds between metrics summary logs.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(default_rviz_config),
            description="RViz configuration file to use when enable_rviz is true.",
        ),
        Node(
            package="rosmaster_a1",
            executable="vehicle_simulator_node",
            parameters=[route_config],
            output="screen",
        ),
        Node(
            package="rosmaster_a1",
            executable="pure_pursuit_controller_node",
            parameters=[route_config],
            output="screen",
        ),
        Node(
            package="rosmaster_a1",
            executable="path_planner_node",
            parameters=[route_config],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="rosmaster_a1",
            executable="metrics_logger_node",
            condition=IfCondition(enable_metrics),
            parameters=[
                route_config,
                {
                    "log_dir": metrics_log_dir,
                    "summary_period_s": metrics_summary_period_s,
                },
            ],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            condition=IfCondition(enable_rviz),
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])