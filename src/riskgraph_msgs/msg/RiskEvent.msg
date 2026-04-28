# A risk event emitted by an adapter or the synthetic publisher.
# header.stamp is the event time; header.frame_id is the spatial frame the
# pose is expressed in (typically "map").

std_msgs/Header header
string event_id                    # uuid4 string; assigned by the emitter
geometry_msgs/Point position       # robot or hazard location at event time, in header.frame_id
string segment_id                  # optional pre-assigned route segment id; "" lets the memory node spatial-join
RiskFactor[] factors               # one or more contributing factors
float32 confidence                 # [0,1] — confidence the event is real
