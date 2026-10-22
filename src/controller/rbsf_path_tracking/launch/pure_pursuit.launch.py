"""
Controller slot: Pure Pursuit tracking of planning/trajectory.

Follows the component launch contract (docs/DESIGN_PHILOSOPHY.md §4.2): takes robot, platform,
track and use_sim_time, and runs rbsf_path_tracking's path_tracker (steering_law: pure_pursuit) as
/<robot>/pure_pursuit. Parameters come from config/pure_pursuit_real.yaml on the real car and
config/pure_pursuit_sim.yaml on the simulators; params_file replaces them and speed overrides the
top speed of the speed profile.

    Subscribes  /<robot>/localization/odom, /<robot>/planning/trajectory
    Publishes   /<robot>/drive_gate/auto_cmd
                /<robot>/pure_pursuit/target
                /<robot>/pure_pursuit/trajectory (the line with the speed profile it drives)
"""

from rbsf_path_tracking.launch_common import launch_description


def generate_launch_description():
    return launch_description('pure_pursuit')
