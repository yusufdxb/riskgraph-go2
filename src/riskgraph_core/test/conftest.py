import sys
from pathlib import Path

# Allow tests to run with bare `pytest src/riskgraph_core/test` from repo root,
# without having to source the ament install tree first.
PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))
