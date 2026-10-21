"""
Registry of platforms, deployments and slot implementations (docs/DESIGN_PHILOSOPHY.md §5).

Behaviours pick one implementation per functional slot by name. Each name maps to the
(package, launch file) that implements the slot's contract. Adding an implementation is one entry
here, and every behaviour can use it. No ROS imports: launch helpers live in compose.py.
"""

SLOTS = ('mapping', 'localization', 'perception', 'planning', 'controller')
PLATFORMS = ('real', 'mvsim', 'f1tenth')
DEPLOYMENTS = ('local', 'distributed')

IMPLEMENTATIONS = {
    'mapping': {
        'identity': ('rbsf_identity', 'mapping.launch.py'),
        'slam_toolbox': ('rbsf_dual_slam', 'slam_toolbox.launch.py'),
        'cartographer': ('rbsf_dual_slam', 'cartographer.launch.py'),
        'dual_slam': ('rbsf_dual_slam', 'dual_slam.launch.py'),
        'map_server': ('rbsf_map_manager', 'map_server.launch.py'),
    },
    'localization': {
        'identity': ('rbsf_identity', 'localization.launch.py'),
        'particle_filter': ('particle_filter', 'particle_filter.launch.py'),
    },
    'perception': {
        'identity': ('rbsf_identity', 'perception.launch.py'),
    },
    'planning': {
        'identity': ('rbsf_identity', 'planning.launch.py'),
    },
    'controller': {
        'identity': ('rbsf_identity', 'controller.launch.py'),
        'wall_following': ('rbsf_wall_following', 'wall_following.launch.py'),
        'follow_the_gap': ('rbsf_follow_the_gap', 'follow_the_gap.launch.py'),
        'pure_pursuit': ('rbsf_path_tracking', 'pure_pursuit.launch.py'),
        'map': ('rbsf_path_tracking', 'map.launch.py'),
    },
}

# Who owns map -> <ns>/odom (docs/DESIGN_PHILOSOPHY.md §3.2): SLAM and identity mappers do; a
# known-map mapper does not, and then a map localizer must.
SLAM_MAPPERS = ('slam_toolbox', 'cartographer', 'dual_slam')
KNOWN_MAP_MAPPERS = ('map_server',)
MAP_LOCALIZERS = ('particle_filter',)
# Controllers that steer from the map-frame pose (localization/odom) rather than the scan.
MAP_FRAME_CONTROLLERS = ('pure_pursuit', 'map')

# Track of each platform when the behaviour gets none. The simulators need a world; the real car
# drives an unknown track.
DEFAULT_TRACK = {'real': '', 'mvsim': 'levine', 'f1tenth': 'levine'}

# The simulators run in real time and stamp their output with wall time (MVSim's adapter, and the
# gym bridge unless use_sim_time_bridge is set), so the whole stack uses wall time for now.
USE_SIM_TIME = {'real': False, 'mvsim': False, 'f1tenth': False}


def resolve_track(platform, track):
    """Return `track` if given, else the platform's default track (none on real)."""
    return track or DEFAULT_TRACK[platform]


def check_choice(kind, value, valid):
    """Raise ValueError naming the valid options if `value` is not one of `valid`."""
    if value not in valid:
        raise ValueError(f"Unknown {kind} '{value}'. Valid options: {', '.join(valid)}")


def validate(platform, deployment, slots):
    """Check platform, deployment and the {slot: implementation} selection (all five slots)."""
    check_choice('platform', platform, PLATFORMS)
    check_choice('deployment', deployment, DEPLOYMENTS)
    missing = [slot for slot in SLOTS if slot not in slots]
    if missing:
        raise ValueError(f"Missing slot(s): {', '.join(missing)}. Every app fills all five.")
    for slot, name in slots.items():
        check_choice('slot', slot, SLOTS)
        check_choice(f'{slot} implementation', name, tuple(IMPLEMENTATIONS[slot]))
    check_map_frame(platform, slots)


def check_map_frame(platform, slots):
    """
    Raise ValueError unless exactly one slot owns map -> <ns>/odom and map-frame consumers get one.

    A map localizer needs a known-map mapper, and a known-map mapper needs a map localizer (else
    two or no TF owners). On the real car identity mapping means map = odom, so a controller that
    follows the map frame needs a SLAM mapper or a map localizer to place the car on the track.
    """
    mapping, localization = slots['mapping'], slots['localization']
    if (localization in MAP_LOCALIZERS) != (mapping in KNOWN_MAP_MAPPERS):
        raise ValueError(
            f'mapping:={mapping} with localization:={localization}: a known-map mapping '
            f"({', '.join(KNOWN_MAP_MAPPERS)}) pairs with a map localization "
            f"({', '.join(MAP_LOCALIZERS)}), which owns map -> odom; other mappings own it "
            'themselves and need localization:=identity.')
    if (platform == 'real' and slots['controller'] in MAP_FRAME_CONTROLLERS
            and localization not in MAP_LOCALIZERS and mapping not in SLAM_MAPPERS):
        raise ValueError(
            f"controller:={slots['controller']} follows the map frame, but on the real car "
            f'mapping:={mapping} + localization:={localization} cannot place it on the track. '
            f'Use mapping:={KNOWN_MAP_MAPPERS[0]} localization:={MAP_LOCALIZERS[0]}.')
