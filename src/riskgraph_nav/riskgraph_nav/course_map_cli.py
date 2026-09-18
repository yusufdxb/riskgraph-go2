"""Regenerate the course map (PGM + YAML) from an experiment file.

    ros2 run riskgraph_nav riskgraph_generate_course_map --experiment <file> [--out <map.yaml>]

Prints the resulting map id. Run it in the SOURCE tree and rebuild; the map
id changes whenever the course geometry changes, which starts a new database.
"""
from __future__ import annotations

import argparse
import sys


def main(argv=None) -> int:
    from riskgraph_core.experiment import load_experiment, write_course_map
    from riskgraph_core.map_identity import compute_map_id
    from .paths import default_experiment_file
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--experiment", default=default_experiment_file())
    ap.add_argument("--out", default=None, help="map yaml path (default: the experiment's map.yaml)")
    a = ap.parse_args(argv)
    exp = load_experiment(a.experiment)
    y, p = write_course_map(exp, a.out)
    print(f"wrote {y}\nwrote {p}\nmap id: {compute_map_id(y, exp.anchor)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
