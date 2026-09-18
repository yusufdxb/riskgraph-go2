"""SVG rendering of a trial: map, risk grid, planned and executed routes.

Dependency free on purpose: the payload has no internet, so the evidence
bundle cannot rely on matplotlib being installed there. The SVG is a
convenience view; the machine-readable numbers live in the JSON files next
to it.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple
from xml.sax.saxutils import escape

from .risk_field import GridInfo

XY = Tuple[float, float]


def render_routes_svg(info: GridInfo, occupancy: Sequence[int], risk: Optional[Sequence[int]],
                      routes: List[Dict], events: List[Dict], title: str,
                      px_per_m: float = 80.0) -> str:
    """routes: [{"name", "points": [(x,y)], "color", "dashed": bool}]
    events: [{"id", "x", "y", "label"}]."""
    w_m = info.width * info.resolution
    h_m = info.height * info.resolution
    W = int(w_m * px_per_m)
    H = int(h_m * px_per_m) + 60
    cell = info.resolution * px_per_m

    def sx(x: float) -> float:
        return (x - info.origin_x) * px_per_m

    def sy(y: float) -> float:
        return 40 + (info.origin_y + h_m - y) * px_per_m

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
           f'viewBox="0 0 {W} {H}" font-family="monospace" font-size="12">',
           f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
           f'<text x="8" y="18" font-size="14">{escape(title)}</text>']
    for iy in range(info.height):
        for ix in range(info.width):
            idx = iy * info.width + ix
            x0 = sx(info.origin_x + ix * info.resolution)
            y0 = sy(info.origin_y + (iy + 1) * info.resolution)
            if occupancy[idx] >= 65:
                out.append(f'<rect x="{x0:.1f}" y="{y0:.1f}" width="{cell:.2f}" '
                           f'height="{cell:.2f}" fill="#333333"/>')
            elif risk is not None and risk[idx] > 0:
                a = min(0.85, 0.15 + risk[idx] / 100.0)
                out.append(f'<rect x="{x0:.1f}" y="{y0:.1f}" width="{cell:.2f}" '
                           f'height="{cell:.2f}" fill="#d9480f" fill-opacity="{a:.2f}"/>')
    for r in routes:
        pts = r.get("points") or []
        if len(pts) < 2:
            continue
        d = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in pts)
        dash = ' stroke-dasharray="6,4"' if r.get("dashed") else ""
        out.append(f'<polyline points="{d}" fill="none" stroke="{r.get("color", "#1c7ed6")}" '
                   f'stroke-width="3"{dash}/>')
    for e in events:
        out.append(f'<circle cx="{sx(e["x"]):.1f}" cy="{sy(e["y"]):.1f}" r="6" fill="#c92a2a" '
                   f'stroke="#000" stroke-width="1"/>')
        out.append(f'<text x="{sx(e["x"]) + 8:.1f}" y="{sy(e["y"]) - 8:.1f}">'
                   f'{escape(str(e.get("label", e.get("id", ""))))}</text>')
    ly = 32
    lx = 8
    for r in routes:
        dash = ' stroke-dasharray="6,4"' if r.get("dashed") else ""
        out.append(f'<line x1="{lx}" y1="{ly}" x2="{lx + 24}" y2="{ly}" '
                   f'stroke="{r.get("color", "#1c7ed6")}" stroke-width="3"{dash}/>')
        out.append(f'<text x="{lx + 28}" y="{ly + 4}">{escape(r["name"])}</text>')
        lx += 40 + 8 * len(r["name"])
    out.append("</svg>")
    return "\n".join(out)
