import sys
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parent
for p in (PKG, SRC / "riskgraph_core", SRC / "riskgraph_memory"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
