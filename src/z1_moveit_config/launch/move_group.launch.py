#!/usr/bin/env python3
"""Start the MoveIt 2 move_group node for the Z1 arm.

Expects a running robot_state_publisher (publishes /robot_description) and the
ros2_control controllers from z1_bringup; `demo.launch.py` starts all of it.

`use_gripper:=false` builds the arm-only model (no gripper links in the URDF,
arm-only SRDF and controller mapping), `use_sim_time:=true` is needed when the
controllers run inside the Gazebo Sim server.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from z1_moveit_config.moveit_config import build_moveit_config


def _launch_setup(context, *args, **kwargs):
    use_gripper = context.launch_configurations.get("use_gripper", "true") == "true"
    use_sim_time = context.launch_configurations.get("use_sim_time", "false") == "true"

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            build_moveit_config(use_gripper=use_gripper).to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    return [move_group]


def generate_launch_description() -> LaunchDescription:
    declared_args = [
        DeclareLaunchArgument(
            "use_gripper",
            default_value="true",
            choices=["true", "false"],
            description="Build the model with the gripper (must match the URDF)",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Use the Gazebo /clock instead of the system clock",
        ),
    ]

    return LaunchDescription(declared_args + [OpaqueFunction(function=_launch_setup)])
