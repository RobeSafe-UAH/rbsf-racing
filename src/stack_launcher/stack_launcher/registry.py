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
    },
    'localization': {
        'identity': ('rbsf_identity', 'localization.launch.py'),
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
    },
}

# The simulators run in real time and stamp their output with wall time (MVSim's adapter, and the
# gym bridge unless use_sim_time_bridge is set), so the whole stack uses wall time for now.
USE_SIM_TIME = {'real': False, 'mvsim': False, 'f1tenth': False}


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
