"""Integration tests run REAL ROS 2 processes (memory node, localization,
map_server, planner_server, controller_server...) on an isolated DDS domain.
They are skipped when ROS 2 Humble, Nav2 or this workspace is not available.
"""
import os
import shutil

import pytest


def pytest_collection_modifyitems(config, items):
    ok = bool(os.environ.get("AMENT_PREFIX_PATH")) and shutil.which("ros2") is not None
    if ok:
        try:
            from ament_index_python.packages import get_package_prefix
            for p in ("nav2_planner", "riskgraph_nav", "riskgraph_memory"):
                get_package_prefix(p)
        except Exception:
            ok = False
    if not ok:
        skip = pytest.mark.skip(reason="needs a sourced ROS 2 Humble + Nav2 + this workspace")
        for it in items:
            it.add_marker(skip)
