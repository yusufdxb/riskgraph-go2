# Score for a single candidate route. Lower total_cost = preferred.
# The breakdown fields are kept so the explainer can cite them directly.

string route_id
float32 total_cost
float32 geometry_cost              # weight * length
float32 semantic_cost              # weight * semantic objective penalty
float32 risk_cost                  # weight * sum of segment risks
string[] dominant_segment_ids      # segments with the highest risk contribution
string[] dominant_factor_categories # categories driving the risk contribution (e.g. ["SLIP","SAFETY"])
