"""Integration launch — wire RiskGraph into a live Go2 stack.

Brings up the core nodes plus optional adapters for upstream topics:
- /go2/safety_alert (go2_msgs/SafetyAlert)
- /helix/faults     (helix_msgs/FaultEvent)
- /tactile/slip_state (std_msgs/Bool)

Each adapter is gated by its own launch argument so the stack still starts
when an upstream package is not installed.

Usage:
    ros2 launch riskgraph_bringup integration.launch.py \\
        enable_safety_adapter:=true \\
        enable_helix_adapter:=true \\
        enable_tactile_adapter:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("riskgraph_bringup")
    config_yaml = os.path.join(bringup_share, "config", "default.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("enable_safety_adapter",  default_value="false"),
        DeclareLaunchArgument("enable_helix_adapter",   default_value="false"),
        DeclareLaunchArgument("enable_tactile_adapter", default_value="false"),

        Node(
            package="riskgraph_memory",
            executable="riskgraph_memory_node",
            name="riskgraph_memory",
            parameters=[config_yaml],
            output="screen",
        ),
        Node(
            package="riskgraph_planner",
            executable="riskgraph_planner_node",
            name="riskgraph_planner",
            parameters=[config_yaml],
            output="screen",
        ),
        Node(
            package="riskgraph_explainer",
            executable="riskgraph_explainer_node",
            name="riskgraph_explainer",
            parameters=[config_yaml],
            output="screen",
        ),
        Node(
            package="riskgraph_memory",
            executable="riskgraph_safety_adapter",
            name="riskgraph_safety_adapter",
            parameters=[config_yaml],
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_safety_adapter")),
        ),
        Node(
            package="riskgraph_memory",
            executable="riskgraph_helix_adapter",
            name="riskgraph_helix_adapter",
            parameters=[config_yaml],
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_helix_adapter")),
        ),
        Node(
            package="riskgraph_memory",
            executable="riskgraph_tactile_adapter",
            name="riskgraph_tactile_adapter",
            parameters=[config_yaml],
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_tactile_adapter")),
        ),
    ])
