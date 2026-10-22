# One waypoint of a planned trajectory: where to drive and how fast.

# Position in the trajectory's frame (m). z is unused by the 2D stack.
geometry_msgs/Point point

# Target speed at this waypoint (m/s). Controllers may drive slower (their own limits), never
# faster. NaN means the planner sets no speed: the controller chooses it.
float64 speed
