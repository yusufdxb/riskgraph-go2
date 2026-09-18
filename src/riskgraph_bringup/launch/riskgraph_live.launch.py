"""RiskGraph live launch: risk memory + route scoring + explainer.

Brings up ONLY RiskGraph (no navigation, no motion). The database path and
map identity are computed once, here, from the experiment file, and passed
as absolute parameters to every node, so no two nodes can open different
files. The computed values are printed at launch.

    ros2 launch riskgraph_bringup riskgraph_live.launch.py run_mode:=live

run_mode has NO default on purpose: live | rehearsal | replay | test. It is
stored in the database and a database created in one mode refuses to open in
another, so replay or rehearsal rows can never land in a live trial's file.

Optional adapters (off by default; the canonical trial does not need them):
enable_safety_adapter, enable_helix_adapter, enable_tactile_adapter.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from riskgraph_nav.paths import DEFAULT_DB_ROOT, DEFAULT_DB_TAG, default_experiment_file, resolve


def _setup(context, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    run_mode = lc("run_mode")
    if run_mode not in ("live", "rehearsal", "replay", "test"):
        raise RuntimeError(f"run_mode must be live|rehearsal|replay|test, got {run_mode!r}")
    r = resolve(lc("experiment"), lc("store_path") or None, lc("db_root"), lc("db_tag"))
    use_sim_time = lc("use_sim_time").lower() == "true"
    common = {"use_sim_time": use_sim_time}
    memory_params = dict(common, store_path=r.store_path, run_mode=run_mode, map_id=r.map_id,
                         experiment_file=r.experiment.path, target_frame="map")
    adapter_params = dict(common, odom_topic=lc("odom_topic"), pose_max_age_s=0.5,
                          output_topic="/riskgraph/risk_events")
    actions = [
        LogInfo(msg=f"[riskgraph] run_mode={run_mode} map_id={r.map_id}"),
        LogInfo(msg=f"[riskgraph] database (absolute): {r.store_path}"),
        LogInfo(msg=f"[riskgraph] experiment: {r.experiment.path}"),
        Node(package="riskgraph_memory", executable="riskgraph_memory_node",
             name="riskgraph_memory", parameters=[memory_params], output="screen"),
        Node(package="riskgraph_planner", executable="riskgraph_planner_node",
             name="riskgraph_planner",
             parameters=[dict(common, store_path=r.store_path, expected_map_id=r.map_id,
                              weight_geometry=1.0, weight_semantic=1.0, weight_risk=4.0,
                              decay_half_life_s=r.experiment.risk_params.decay_half_life_s)],
             output="screen"),
        Node(package="riskgraph_explainer", executable="riskgraph_explainer_node",
             name="riskgraph_explainer",
             parameters=[dict(common, store_path=r.store_path)], output="screen"),
    ]
    for kind in ("safety", "helix", "tactile"):
        actions.append(Node(
            package="riskgraph_memory", executable=f"riskgraph_{kind}_adapter",
            name=f"riskgraph_{kind}_adapter", parameters=[adapter_params], output="screen",
            condition=IfCondition(LaunchConfiguration(f"enable_{kind}_adapter"))))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("run_mode", description="live | rehearsal | replay | test (required)"),
        DeclareLaunchArgument("experiment", default_value=default_experiment_file()),
        DeclareLaunchArgument("store_path", default_value="",
                              description="absolute DB path; empty = <db_root>/<map_id>/<db_tag>.sqlite"),
        DeclareLaunchArgument("db_root", default_value=DEFAULT_DB_ROOT),
        DeclareLaunchArgument("db_tag", default_value=DEFAULT_DB_TAG),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("odom_topic", default_value="/utlidar/robot_odom"),
        DeclareLaunchArgument("enable_safety_adapter", default_value="false"),
        DeclareLaunchArgument("enable_helix_adapter", default_value="false"),
        DeclareLaunchArgument("enable_tactile_adapter", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
