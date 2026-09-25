"""Shared MoveIt 2 configuration for the Z1.

Kept in one module so that every launch file builds the exact same parameter
set (URDF, SRDF, kinematics, limits, controller mapping, OMPL).

`use_gripper=False` mirrors `z1_description`'s `use_gripper:=false`: the URDF has
no gripper links then, so the SRDF, the controller mapping and the joint limits
are switched to their arm-only variants as well.
"""

import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

DESCRIPTION_PACKAGE = "z1_description"

# Arm-only variants, used when use_gripper is False.
SRDF_ARM_ONLY = "config/z1_no_gripper.srdf"
CONTROLLERS_ARM_ONLY = "config/moveit_controllers_no_gripper.yaml"


def _xacro_args(use_gripper: bool) -> dict:
    # `hardware_plugin` is left at its default: MoveIt only reads the URDF, it
    # never loads the hardware, and the arm's command interface is `position`
    # on the real-arm backend, which is what moveit_controllers.yaml maps.
    return {
        "use_gripper": "true" if use_gripper else "false",
        "use_gazebo": "false",
    }


def build_moveit_config(use_gripper: bool = True):
    """Return the MoveIt parameter set of the Z1 (pulled from two packages)."""
    xacro_file = os.path.join(
        get_package_share_directory(DESCRIPTION_PACKAGE), "urdf", "z1.urdf.xacro"
    )
    configs = (
        MoveItConfigsBuilder("z1", package_name="z1_moveit_config")
        .robot_description(file_path=xacro_file, mappings=_xacro_args(use_gripper))
        .robot_description_semantic(
            file_path="config/z1.srdf" if use_gripper else SRDF_ARM_ONLY
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(
            file_path=(
                "config/moveit_controllers.yaml" if use_gripper else CONTROLLERS_ARM_ONLY
            )
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    if not use_gripper:
        # joint_limits.yaml is shared; drop the joint that is not in the model.
        # configs.joint_limits looks like {"<robot>_planning": {"joint_limits": {...}}}
        for limits in configs.joint_limits.values():
            limits.get("joint_limits", {}).pop("jointGripper", None)

    return configs
