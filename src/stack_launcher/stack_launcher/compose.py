"""Launch helpers shared by the behaviours: include the platform side and the five slots."""

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from stack_launcher.registry import IMPLEMENTATIONS, SLOTS


def launch_file(package, name):
    """Path of share/<package>/launch/<name>, with a clear error if the package is missing."""
    try:
        share = get_package_share_directory(package)
    except PackageNotFoundError:
        raise RuntimeError(
            f"Package '{package}' is not installed. Simulator packages only exist in the dev "
            'image (FLAVOR=dev); run `cb` if it is a workspace package.') from None
    return os.path.join(share, 'launch', name)


def include(package, name, arguments):
    """Include share/<package>/launch/<name>, passing `arguments` as strings."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file(package, name)),
        launch_arguments={key: str(value) for key, value in arguments.items()}.items())


def slot_arguments(slot, robot, platform, track, use_sim_time, extra=None):
    """
    Launch arguments of one slot: the component contract plus the behaviour's `extra[slot]`.

    Extra arguments are implementation options a behaviour exposes (e.g. the controller's
    wall_side). An implementation that does not declare one simply ignores it.
    """
    contract = {'robot': robot, 'platform': platform, 'track': track,
                'use_sim_time': str(use_sim_time).lower()}
    return {**contract, **(extra or {}).get(slot, {})}


def include_slots(slots, robot, platform, track, use_sim_time, extra=None):
    """One include per slot, in SLOTS order, with the contract and per-slot extra arguments."""
    return [include(*IMPLEMENTATIONS[slot][slots[slot]],
                    slot_arguments(slot, robot, platform, track, use_sim_time, extra))
            for slot in SLOTS]
