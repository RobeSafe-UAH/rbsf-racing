"""Launch arguments shared by the rbsf_dual_slam launch files."""

from launch.actions import DeclareLaunchArgument

PLATFORMS = ('real', 'mvsim', 'f1tenth')


def slam_arguments():
    """Declare the component contract (DESIGN_PHILOSOPHY §4.2), primary, map_saver, output_dir."""
    return [
        DeclareLaunchArgument('robot', default_value='r0', description='Car namespace.'),
        DeclareLaunchArgument(
            'platform', default_value='mvsim', description='real, mvsim or f1tenth.'),
        DeclareLaunchArgument('track', default_value='', description='rbsf_worlds track.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'primary', default_value='true',
            description='Own /map and map -> <robot>/odom (false: secondary SLAM, no TF).'),
        DeclareLaunchArgument(
            'map_saver', default_value='true',
            description='Start a map_saver for this SLAM (dual_slam starts one for both).'),
        DeclareLaunchArgument(
            'output_dir', default_value='/ros2_ws/maps', description='Where maps are saved.'),
    ]


def check_platform(platform):
    if platform not in PLATFORMS:
        raise RuntimeError(f"Unknown platform '{platform}'. Valid options: {list(PLATFORMS)}")
