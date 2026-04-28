# A topological segment of a route between two waypoints.
# Segment ids are stable across runs once assigned; the planner uses them
# as the join key into the persistent risk store.

string segment_id                  # stable id, e.g. "seg_004A"
geometry_msgs/Point start
geometry_msgs/Point end
float32 length_m                   # geometric length, meters
string semantic_label              # optional, e.g. "hallway-east", "kitchen-doorway"
