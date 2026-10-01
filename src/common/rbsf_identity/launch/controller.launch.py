"""
Identity controller: launches nothing for now.

Follows the component launch contract (docs/DESIGN_PHILOSOPHY.md §4.2). The neutral output the
design asks for (§3.3) comes once rbsf_msgs and a consumer of this slot exist.
"""

from launch import LaunchDescription
from rbsf_identity.contract import contract_arguments


def generate_launch_description():
    return LaunchDescription(contract_arguments())
