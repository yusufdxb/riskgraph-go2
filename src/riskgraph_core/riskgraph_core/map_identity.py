"""Map identity: one stable string per (map image, map metadata, anchor).

Stored risk positions are only meaningful against the map they were
recorded in. The id below is what every RiskGraph process, the preflight and
the trial runner compute from the same files, and what the SQLite store is
bound to on first write. Change the image, the resolution, the origin, or
where the physical start marker sits in the map, and the id changes, so an
old database refuses to open instead of silently projecting old risk onto a
different floor.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import yaml

from .risk_field import GridInfo


class MapIdentityInputError(ValueError):
    """The map YAML or image is missing or malformed."""


@dataclass(frozen=True)
class MapDescription:
    yaml_path: str
    image_path: str
    resolution: float
    origin: Tuple[float, float, float]
    negate: int
    occupied_thresh: float
    free_thresh: float
    mode: str
    width: Optional[int]
    height: Optional[int]
    image_sha256: str

    def grid_info(self) -> GridInfo:
        if self.width is None or self.height is None:
            raise MapIdentityInputError(
                f"cannot derive grid size from {self.image_path} (only PGM is parsed)")
        if abs(self.origin[2]) > 1e-9:
            raise MapIdentityInputError("map origin yaw must be 0")
        return GridInfo(resolution=self.resolution, width=self.width, height=self.height,
                        origin_x=self.origin[0], origin_y=self.origin[1])


def _pgm_size(data: bytes) -> Optional[Tuple[int, int]]:
    """Width/height from a binary (P5) or ASCII (P2) PGM header, else None."""
    if not data.startswith((b"P5", b"P2")):
        return None
    tokens = []
    i = 2
    n = len(data)
    while len(tokens) < 3 and i < n:
        c = data[i:i + 1]
        if c == b"#":
            while i < n and data[i:i + 1] not in (b"\n", b"\r"):
                i += 1
        elif c.isspace():
            i += 1
        else:
            j = i
            while j < n and not data[j:j + 1].isspace() and data[j:j + 1] != b"#":
                j += 1
            tokens.append(data[i:j])
            i = j
    try:
        return int(tokens[0]), int(tokens[1])
    except (IndexError, ValueError):
        return None


def describe_map(yaml_path: str) -> MapDescription:
    if not yaml_path or not os.path.isfile(yaml_path):
        raise MapIdentityInputError(f"map yaml not found: {yaml_path!r}")
    with open(yaml_path, "r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh) or {}
    if not isinstance(raw, dict) or "image" not in raw:
        raise MapIdentityInputError(f"{yaml_path}: not a map_server yaml (no 'image')")
    image = str(raw["image"])
    image_path = image if os.path.isabs(image) else os.path.join(
        os.path.dirname(os.path.abspath(yaml_path)), image)
    if not os.path.isfile(image_path):
        raise MapIdentityInputError(f"{yaml_path}: image not found: {image_path}")
    with open(image_path, "rb") as fh:
        data = fh.read()
    try:
        resolution = float(raw["resolution"])
        origin = tuple(float(v) for v in raw["origin"])
    except (KeyError, TypeError, ValueError) as exc:
        raise MapIdentityInputError(f"{yaml_path}: bad resolution/origin ({exc})") from exc
    if len(origin) != 3 or not all(math.isfinite(v) for v in origin) or not (
            resolution > 0 and math.isfinite(resolution)):
        raise MapIdentityInputError(f"{yaml_path}: resolution/origin not finite")
    size = _pgm_size(data)
    return MapDescription(
        yaml_path=os.path.abspath(yaml_path),
        image_path=os.path.abspath(image_path),
        resolution=resolution,
        origin=origin,  # type: ignore[arg-type]
        negate=int(raw.get("negate", 0)),
        occupied_thresh=float(raw.get("occupied_thresh", 0.65)),
        free_thresh=float(raw.get("free_thresh", 0.25)),
        mode=str(raw.get("mode", "trinary")),
        width=size[0] if size else None,
        height=size[1] if size else None,
        image_sha256=hashlib.sha256(data).hexdigest(),
    )


def compute_map_id(yaml_path: str, anchor: Optional[Dict[str, Any]] = None,
                   name: Optional[str] = None) -> str:
    """``<name>-<12 hex>`` over the map content and the anchor definition.

    ``anchor`` is the physical start marker's pose in the map (``{"marker":
    "A", "x": .., "y": .., "yaw": ..}``). File paths are deliberately NOT part
    of the id: the same map copied to the robot must keep its identity.
    """
    d = describe_map(yaml_path)
    canonical = {
        "image_sha256": d.image_sha256,
        "resolution": round(d.resolution, 9),
        "origin": [round(v, 9) for v in d.origin],
        "negate": d.negate,
        "occupied_thresh": round(d.occupied_thresh, 9),
        "free_thresh": round(d.free_thresh, 9),
        "mode": d.mode,
        "anchor": _canonical_anchor(anchor),
    }
    digest = hashlib.sha256(json.dumps(canonical, sort_keys=True).encode()).hexdigest()
    stem = name or os.path.splitext(os.path.basename(yaml_path))[0]
    safe = "".join(ch if (ch.isalnum() or ch in "-_") else "_" for ch in stem)
    return f"{safe}-{digest[:12]}"


def _canonical_anchor(anchor: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if anchor is None:
        return None
    try:
        return {
            "marker": str(anchor.get("marker", "")),
            "x": round(float(anchor["x"]), 6),
            "y": round(float(anchor["y"]), 6),
            "yaw": round(float(anchor["yaw"]), 6),
        }
    except (KeyError, TypeError, ValueError) as exc:
        raise MapIdentityInputError(f"anchor must have numeric x, y, yaw ({exc})") from exc
