"""Navigation side of the live trial: anchored localization + Nav2 servers.

    ros2 launch riskgraph_bringup riskgraph_nav_live.launch.py

Start this with the robot STANDING STILL ON MARKER A, facing marker B: the
map frame is anchored on the first stationary odometry window (anchor_mode
auto). If the robot was not on A, stop this launch, place it, relaunch.

Nodes: riskgraph_localization (TF odom->base_link restamped on this host's
clock, map->odom from the anchor), map_server (course map from the
experiment file), planner_server (global costmap = map + RiskGraph risk
layer), controller_server (RPP), velocity_smoother, lifecycle manager.

Motion output: controller -> /cmd_vel_nav -> velocity_smoother ->
/nav/cmd_vel. /nav/cmd_vel is the only input of riskgraph_sport_sink (started
separately, docs/HW_VERIFICATION.md section 5); nothing here talks to the robot. No goal is ever sent by this
launch: motion only happens after the trial runner's typed arming.

Stationary check: ``sink_prefix:=/rg_check`` moves every output of this launch
(/cmd_vel_nav, /nav/cmd_vel, /tf, /tf_static) under that prefix, so the stack can
be brought up on a live robot with nothing reaching the sport sink or the
robot's TF tree. Empty (the default) keeps the real topics.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from riskgraph_nav.paths import default_experiment_file, resolve


def _setup(context, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    r = resolve(lc("experiment"))
    params_file = lc("nav2_params") or os.path.join(
        get_package_share_directory("riskgraph_bringup"), "config", "nav2_live.yaml")
    use_sim_time = lc("use_sim_time").lower() == "true"
    sim = {"use_sim_time": use_sim_time}
    odom = lc("odom_topic")
    prefix = lc("sink_prefix").rstrip("/")
    if prefix and not prefix.startswith("/"):
        raise RuntimeError(f"sink_prefix must be an absolute topic prefix, got {prefix!r}")
    tf = [("/tf", f"{prefix}/tf"), ("/tf_static", f"{prefix}/tf_static")] if prefix else []
    cmd_vel_nav = f"{prefix}/cmd_vel_nav"
    nav_cmd_vel = f"{prefix}/nav/cmd_vel"
    return [
        LogInfo(msg=f"[riskgraph_nav] outputs: {cmd_vel_nav} -> {nav_cmd_vel}"
                    + (f" (STATIONARY CHECK, tf under {prefix})" if prefix else "")),
        LogInfo(msg=f"[riskgraph_nav] map_id={r.map_id} map={r.experiment.map_yaml}"),
        LogInfo(msg=f"[riskgraph_nav] nav2 params: {params_file}"),
        Node(package="riskgraph_nav", executable="riskgraph_localization",
             name="riskgraph_localization", output="screen",
             parameters=[dict(sim, experiment_file=r.experiment.path, odom_topic=odom,
                              anchor_mode=lc("anchor_mode"))],
             remappings=tf),
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen",
             parameters=[params_file, dict(sim, yaml_filename=r.experiment.map_yaml)],
             remappings=tf),
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             output="screen", parameters=[params_file, sim], remappings=tf),
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             output="screen", parameters=[params_file, sim],
             remappings=[("cmd_vel", cmd_vel_nav), ("odom", odom)] + tf),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             name="velocity_smoother", output="screen", parameters=[params_file, sim],
             remappings=[("cmd_vel", cmd_vel_nav), ("cmd_vel_smoothed", nav_cmd_vel)] + tf),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_riskgraph_nav", output="screen",
             parameters=[params_file, sim]),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("experiment", default_value=default_experiment_file()),
        DeclareLaunchArgument("nav2_params", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("odom_topic", default_value="/utlidar/robot_odom"),
        DeclareLaunchArgument("anchor_mode", default_value="auto"),
        DeclareLaunchArgument("sink_prefix", default_value=""),
        OpaqueFunction(function=_setup),
    ])
