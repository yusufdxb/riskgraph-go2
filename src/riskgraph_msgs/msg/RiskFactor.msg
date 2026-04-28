# A single risk factor contributing to a RiskEvent.
# Categories are stable strings so adapters from heterogeneous upstream
# topics (safety alerts, slip flags, audio events, etc.) can normalize into
# a common vocabulary without IDL changes.
#
# Stable category values:
#   SLIP        - locomotion instability / slip / proprio anomaly
#   SAFETY      - go2_msgs/SafetyAlert-shaped events (EMERGENCY_STOP, etc.)
#   DEPTH       - depth sensor hazard (drop, low overhead, narrow passage)
#   AUDIO       - audio anomaly (loud noise, sustained alarm)
#   FAULT       - helix_msgs/FaultEvent-shaped system fault
#   HUMAN       - human-relevant hazard (crowd, occluder)
#   COLLISION   - near-collision / contact event
#   OTHER       - unclassified

string category
float32 severity         # [0,1] — 0=ignore, 1=critical
string source            # human-readable origin, e.g. "go2/safety_alert", "tactile/slip_state"
string detail            # optional free-form context
