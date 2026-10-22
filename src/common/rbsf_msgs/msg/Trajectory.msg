# Planned trajectory, the planning slot's output on planning/trajectory
# (docs/DESIGN_PHILOSOPHY.md §4.1). Transient local.
#
# Waypoints are ordered in the driving direction. A global line (the track's reference line) is a
# closed loop: the last waypoint connects back to the first. No waypoints means "no trajectory",
# which controllers treat as "do not drive".

# frame_id: the map frame.
std_msgs/Header header

Waypoint[] waypoints
