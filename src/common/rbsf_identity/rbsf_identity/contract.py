"""Launch arguments every component implementation takes (docs/DESIGN_PHILOSOPHY.md §4.2)."""

from launch.actions import DeclareLaunchArgument


def contract_arguments():
    """Declare the robot, platform, track and use_sim_time launch arguments."""
    return [
        DeclareLaunchArgument('robot', default_value='r0', description='Car namespace.'),
        DeclareLaunchArgument(
            'platform', default_value='mvsim', description='real, mvsim or f1tenth.'),
        DeclareLaunchArgument('track', default_value='', description='rbsf_worlds track.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]
