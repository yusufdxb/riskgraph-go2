"""Offline synthetic demo launch.

Brings up: memory node + planner + explainer + synthetic publisher.
No upstream Go2 stack required; runs entirely against the bundled scenario fixture.

Usage:
    ros2 launch riskgraph_bringup demo_offline.launch.py
    ros2 launch riskgraph_bringup demo_offline.launch.py scenario:=/path/to/fixture.json
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("riskgraph_bringup")
    config_yaml = os.path.join(bringup_share, "config", "default.yaml")

    demo_share = get_package_share_directory("riskgraph_demo")
    default_scenario = os.path.join(
        demo_share, "fixtures", "scenario_glossy_hallway.json"
    )

    scenario = LaunchConfiguration("scenario")
    publish_period = LaunchConfiguration("publish_period_s")

    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value=default_scenario),
        DeclareLaunchArgument("publish_period_s", default_value="0.2"),

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
            package="riskgraph_demo",
            executable="riskgraph_synthetic_publisher",
            name="riskgraph_synthetic_publisher",
            parameters=[{
                "scenario_path": scenario,
                "publish_period_s": publish_period,
                "output_topic": "/riskgraph/risk_events",
            }],
            output="screen",
        ),
    ])
