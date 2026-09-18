"""What time is an event, given clocks that are known to disagree.

Measured on the lab GO2: the robot's own message stamps read months in the
past, and the payload Jetson boots with no RTC. An event stamp can therefore
be off by far more than any decay half life. Rather than let one wrong clock
decide whether a hazard counts, the ingesting node keeps the stamp when it is
plausible and otherwise falls back to its own receipt time, and records that
it did so.
"""
from __future__ import annotations

import math
from typing import Tuple

#: Largest |event stamp - receipt time| accepted as the event's own time.
DEFAULT_MAX_SKEW_S = 300.0


def effective_event_time(stamp_s: float, receipt_s: float,
                         max_skew_s: float = DEFAULT_MAX_SKEW_S) -> Tuple[float, str]:
    """Return ``(time, note)``; ``note`` is empty when the stamp was kept."""
    try:
        s = float(stamp_s)
    except (TypeError, ValueError):
        s = float("nan")
    r = float(receipt_s)
    if not math.isfinite(s) or s <= 0.0:
        return r, "stamp_unset_used_receipt"
    skew = r - s
    if abs(skew) > max_skew_s:
        return r, f"stamp_skew_{skew:+.1f}s_used_receipt"
    return s, ""
