import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parents[1]
CORE_ROOT = PKG_ROOT.parent / "riskgraph_core"
for p in (PKG_ROOT, CORE_ROOT):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
