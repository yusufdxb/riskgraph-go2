"""Single source of truth for experiment, map identity and database paths.

The launch files, the preflight, the trial runner and the status CLI all call
these functions, so "which database file does this trial use" has exactly one
answer: ``<db_root>/<map_id>/<db_tag>.sqlite``, absolute. Two processes can
only disagree if someone passes a different path explicitly, and then the
preflight catches it (every RiskGraph node's ``store_path`` must be equal).
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Optional

from riskgraph_core.experiment import Experiment, load_experiment
from riskgraph_core.map_identity import compute_map_id

DEFAULT_DB_ROOT = "~/.local/share/riskgraph"
DEFAULT_EVIDENCE_ROOT = "~/riskgraph_runs"
DEFAULT_DB_TAG = "trial"


def default_experiment_file() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("riskgraph_bringup"),
                            "config", "experiment", "two_corridor.yaml")
    except Exception:  # not installed: fall back to the source tree
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(os.path.join(
            here, "..", "..", "riskgraph_bringup", "config", "experiment", "two_corridor.yaml"))


@dataclass(frozen=True)
class Resolved:
    experiment: Experiment
    map_id: str
    store_path: str


def resolve(experiment_file: Optional[str] = None, store_path: Optional[str] = None,
            db_root: Optional[str] = None, db_tag: Optional[str] = None) -> Resolved:
    exp = load_experiment(experiment_file or default_experiment_file())
    map_id = compute_map_id(exp.map_yaml, exp.anchor)
    if store_path:
        path = os.path.abspath(os.path.expanduser(store_path))
    else:
        root = os.path.abspath(os.path.expanduser(db_root or DEFAULT_DB_ROOT))
        tag = db_tag or DEFAULT_DB_TAG
        if not tag.replace("-", "").replace("_", "").isalnum():
            raise ValueError(f"db_tag must be alphanumeric/-/_, got {tag!r}")
        path = os.path.join(root, map_id, f"{tag}.sqlite")
    return Resolved(exp, map_id, path)
