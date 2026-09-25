#!/usr/bin/env python3
"""Visualise the Z1 description in RViz2 (URDF only, no physics, no hardware)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("z1_description")
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "z1.urdf.xacro"])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "z1_display.rviz"])

    declared_args = [
        DeclareLaunchArgument(
            "use_gripper",
            default_value="true",
            choices=["true", "false"],
            description="Attach the Unitree gripper to the arm",
        ),
        DeclareLaunchArgument(
            "use_joint_state_publisher_gui",
            default_value="true",
            choices=["true", "false"],
            description="Start the joint_state_publisher GUI to drag the joints",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Start RViz2",
        ),
    ]

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " use_gripper:=",
                LaunchConfiguration("use_gripper"),
                " hardware_plugin:=mock_components/GenericSystem",
                " use_gazebo:=false",
            ]
        ),
        value_type=str,
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_joint_state_publisher_gui")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ]

    return LaunchDescription(declared_args + nodes)
