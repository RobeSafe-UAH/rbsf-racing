"""
f1tenth_gym_ros bridge with opponents numbered from 0.

Runs the upstream bridge (f1tenth_gym_ros.gym_bridge) unchanged except for its opponent suffix:
upstream names opponents <opp_namespace>, <opp_namespace>2, <opp_namespace>3, ...; here they are
<opp_namespace>0, <opp_namespace>1, ... See naming.py for the full topic layout and
f1tenth_gym.launch.py for how it is applied.

Needs the f1tenth_gym interpreter (RBSF_GYM_PYTHON); the launch file starts it with it.
"""


def opp_suffix(opp_index):
    """Return the suffix of opponent `opp_index` (upstream counts from 1) counting from 0."""
    return str(opp_index - 1)


def main(args=None):
    # Imported here: it pulls in f1tenth_gym, which only the gym interpreter has.
    from f1tenth_gym_ros import gym_bridge as upstream

    upstream.opp_suffix = opp_suffix
    upstream.main(args)


if __name__ == '__main__':
    main()
