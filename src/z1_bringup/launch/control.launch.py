#!/usr/bin/env python3
"""Start ros2_control for the Z1: robot_state_publisher + controller_manager.

No RViz and no Gazebo - this is the reusable core that the visualisation and
MoveIt launch files build on.

`hardware_plugin` selects the ros2_control backend:
  mock_components/GenericSystem      perfect servo, no hardware (default)
  z1_ros2_control/Z1System           the real arm via z1_controller
  gz_ros2_control/GazeboSimSystem    Gazebo Sim, driven by the gz_ros2_control plugin
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from z1_bringup.launch_parts import (
    ARM_CONTROLLERS,
    CONTROLLER_MANAGER,
    GRIPPER_CONTROLLER,
    controller_spawner,
)


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = FindPackageShare("z1_bringup")
    pkg_description = FindPackageShare("z1_description")

    xacro_file = PathJoinSubstitution([pkg_description, "urdf", "z1.urdf.xacro"])
    controllers_file = PathJoinSubstitution([pkg_bringup, "config", "z1_controllers.yaml"])

    use_gripper = LaunchConfiguration("use_gripper")

    declared_args = [
        DeclareLaunchArgument(
            "use_gripper",
            default_value="true",
            choices=["true", "false"],
            description="Attach the Unitree gripper to the arm",
        ),
        DeclareLaunchArgument(
            "hardware_plugin",
            default_value="mock_components/GenericSystem",
            description="ros2_control hardware plugin to load",
        ),
    ]

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " use_gripper:=",
                use_gripper,
                " hardware_plugin:=",
                LaunchConfiguration("hardware_plugin"),
                " use_gazebo:=false",
            ]
        ),
        value_type=str,
    )

    spawner_args = ["--controller-manager", CONTROLLER_MANAGER]

    nodes = [
        # Publishes /robot_description (transient local); ros2_control_node reads
        # the hardware description from that topic.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            output="screen",
            parameters=[controllers_file],
        ),
    ] + [
        controller_spawner(controller, spawner_args) for controller in ARM_CONTROLLERS
    ] + [
        controller_spawner(
            GRIPPER_CONTROLLER, spawner_args, condition=IfCondition(use_gripper)
        ),
    ]

    return LaunchDescription(declared_args + nodes)
