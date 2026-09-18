#!/usr/bin/env bash
# Run all offline-friendly unit tests for RiskGraph-Go2.
# Does not require ROS to be sourced; pure-Python paths only.
# ROS integration tests (real processes, real Nav2) live in tests/integration
# and run with: ./scripts/run_integration_tests.sh
set -euo pipefail
cd "$(dirname "$0")/.."

PYTHONPATH="src/riskgraph_core:src/riskgraph_memory:src/riskgraph_demo:src/riskgraph_nav" \
    python3 -m pytest \
        src/riskgraph_core/test \
        src/riskgraph_memory/test \
        src/riskgraph_demo/test \
        src/riskgraph_nav/test \
        "$@"
