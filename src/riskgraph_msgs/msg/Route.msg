# An ordered list of segments forming a candidate route from start to goal.
std_msgs/Header header
string route_id                    # stable id within a planning round
RouteSegment[] segments
