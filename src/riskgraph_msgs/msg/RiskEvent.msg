# A risk observation emitted by an adapter, the trial runner, or a replay.
#
# header.stamp    event time (the memory node falls back to its receipt time
#                 when the stamp is unset or implausibly skewed, and records it)
# header.frame_id frame `position` is expressed in. "" means "no pose": such an
#                 event is quarantined, never placed on the map. Any other frame
#                 is transformed into the memory node's target frame ("map") with
#                 a timestamped TF lookup; if that fails the event is quarantined.

std_msgs/Header header
string event_id                    # uuid4 string, assigned by the emitter (dedup key)
geometry_msgs/Point position       # robot or hazard location at event time, in header.frame_id
string segment_id                  # optional pre-assigned route segment id; "" lets the memory node spatial-join
RiskFactor[] factors               # one or more contributing factors
float32 confidence                 # [0,1], confidence the event is real

# Provenance. A memory node in live mode quarantines anything that is not
# HARDWARE_DERIVED or OPERATOR_INJECTED.
#   HARDWARE_DERIVED   computed from real robot / ROS measurements
#   OPERATOR_INJECTED  deliberate, explicitly marked test event at a real live pose
#   REPLAY             re-published from a rosbag
#   SYNTHETIC          fixtures, demos, unit tests
#   SIMULATION         simulator or rehearsal robot
string provenance
string source_frame_id             # frame the observation was made in, if the emitter already transformed it
string map_id                      # map identity the emitter asserts ("" = not asserted)
