# Human-readable explanation of a route choice.
# `evidence_event_ids` cite the specific RiskEvents that drove the choice,
# so a reviewer can audit the explanation against the persistent log.

std_msgs/Header header
string route_id
string text                        # template-rendered, deterministic
string[] evidence_event_ids        # RiskEvent.event_id values cited in `text`
