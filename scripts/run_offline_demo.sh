#!/usr/bin/env bash
# Run the deterministic offline demo (no ROS required).
# Loads the bundled glossy_hallway scenario, runs the full risk model, prints
# a summary, and writes machine-readable output to demo_results.json.
set -euo pipefail
cd "$(dirname "$0")/.."

FIXTURE="${1:-src/riskgraph_demo/fixtures/scenario_glossy_hallway.json}"
OUTPUT="${2:-demo_results.json}"

PYTHONPATH="src/riskgraph_core:src/riskgraph_demo" \
    python3 -m riskgraph_demo.offline_demo "$FIXTURE" "$OUTPUT"
