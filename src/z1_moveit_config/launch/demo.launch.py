#!/usr/bin/env python3
"""Full Z1 MoveIt 2 demo: robot bringup + move_group + RViz2.

`use_gazebo:=false` (default) puts the mock hardware (a perfect servo) under
MoveIt. `use_gazebo:=true` puts the Gazebo Sim + ros2_control path of
z1_bringup there instead, and switches move_group and RViz to the Gazebo clock.
`use_gripper:=false` builds the arm-only model (no end effector), which must
match the URDF that z1_bringup generates.

RViz starts with the MoveIt MotionPlanning panel (planning group `arm`), so the
interactive markers, Plan and Execute work out of the box.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from z1_moveit_config.moveit_config import build_moveit_config

# move_group needs /robot_description and the controller actions; give
# robot_state_publisher and the controller spawners a head start. The Gazebo
# path needs more, the model is spawned twice (world + controller_manager).
MOVE_GROUP_DELAY_S = 5.0
GAZEBO_MOVE_GROUP_DELAY_S = 20.0


def _flag(context, name: str, default: str) -> bool:
    return context.launch_configurations.get(name, default) == "true"


def _launch_setup(context, *args, **kwargs):
    use_gripper = _flag(context, "use_gripper", "true")
    use_gazebo = _flag(context, "use_gazebo", "false")
    launch_arg = "true" if use_gripper else "false"

    moveit_config = build_moveit_config(use_gripper=use_gripper)

    if use_gazebo:
        bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("z1_bringup"), "/launch/gazebo_ros2_control.launch.py"]
            ),
            launch_arguments={
                "use_gripper": launch_arg,
                "use_rviz": "false",
                "headless": context.launch_configurations.get("headless", "false"),
            }.items(),
        )
        delay = GAZEBO_MOVE_GROUP_DELAY_S
    else:
        bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("z1_bringup"), "/launch/control.launch.py"]
            ),
            launch_arguments={"use_gripper": launch_arg}.items(),
        )
        delay = MOVE_GROUP_DELAY_S

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("z1_moveit_config"), "/launch/move_group.launch.py"]
        ),
        launch_arguments={
            "use_gripper": launch_arg,
            "use_sim_time": "true" if use_gazebo else "false",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("z1_moveit_config"), "rviz", "moveit.rviz"]
            ),
        ],
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_gazebo}],
    )

    # Branch here instead of using IfCondition(LaunchConfiguration("use_rviz")):
    # the Gazebo bringup is included with use_rviz:=false, and
    # IncludeLaunchDescription leaks its launch arguments into this scope, so a
    # later IfCondition would see "false" and silently skip RViz.
    actions = [bringup, TimerAction(period=delay, actions=[move_group])]
    if _flag(context, "use_rviz", "true"):
        actions.append(rviz)

    return actions


def generate_launch_description() -> LaunchDescription:
    declared_args = [
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Start RViz2 with the MoveIt MotionPlanning display",
        ),
        DeclareLaunchArgument(
            "use_gripper",
            default_value="true",
            choices=["true", "false"],
            description="Build the model with the gripper (must match the URDF)",
        ),
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="false",
            choices=["true", "false"],
            description="Use the Gazebo Sim + ros2_control bringup instead of mock hardware",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            choices=["true", "false"],
            description="Only used with use_gazebo:=true: run gz-sim without its GUI",
        ),
    ]

    return LaunchDescription(declared_args + [OpaqueFunction(function=_launch_setup)])
