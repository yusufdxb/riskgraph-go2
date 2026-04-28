#!/usr/bin/env bash
# Run all offline-friendly tests for RiskGraph-Go2.
# Does not require ROS to be sourced; pure-Python paths only.
set -euo pipefail
cd "$(dirname "$0")/.."

PYTHONPATH="src/riskgraph_core:src/riskgraph_memory:src/riskgraph_demo" \
    python3 -m pytest \
        src/riskgraph_core/test \
        src/riskgraph_memory/test \
        src/riskgraph_demo/test \
        "$@"
