#!/usr/bin/env python3
"""Start ros2_control for the Z1: robot_state_publisher + controller_manager.

No RViz and no Gazebo - this is the reusable core that the visualisation and
MoveIt launch files build on.

`hardware_plugin` selects the ros2_control backend:
  z1_ros2_control/Z1System           the real arm via z1_controller (default)
  gz_ros2_control/GazeboSimSystem    Gazebo Sim, driven by the gz_ros2_control plugin

The `ctrl_*` arguments below are only used by the real-arm backend; they are
passed into the URDF as `<param>` entries of the `<hardware>` block, which is
where ros2_control reads a component's configuration from.
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

# Launch arguments that are forwarded verbatim into the URDF's <hardware> block.
# Keep this list in sync with the `<param name=...>` entries in
# z1_description/urdf/z1_ros2_control.xacro.
HARDWARE_PARAMETERS = [
    ("ctrl_ip", "127.0.0.1", "Host running z1_ctrl (it must be this machine)"),
    ("ctrl_port", "8071", "Port z1_ctrl binds"),
    ("own_port", "8072", "Port this component binds; z1_ctrl sends its state here"),
    (
        "activate_fsm_sequence",
        "JOINTCTRL",
        "Comma-separated FSM states to request when the hardware is activated",
    ),
    (
        "disconnect_timeout_ms",
        "200",
        "Silence from z1_ctrl that is reported as a hardware error",
    ),
    ("temperature_limit", "80", "Motor over-temperature limit in Celsius"),
]


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
            default_value="z1_ros2_control/Z1System",
            description="ros2_control hardware plugin to load",
        ),
    ] + [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in HARDWARE_PARAMETERS
    ]

    xacro_args = ["xacro ", xacro_file, " use_gripper:=", use_gripper]
    xacro_args += [" hardware_plugin:=", LaunchConfiguration("hardware_plugin")]
    xacro_args += [" use_gazebo:=false"]
    for name, _default, _description in HARDWARE_PARAMETERS:
        xacro_args += [" ", name, ":=", LaunchConfiguration(name)]

    robot_description = ParameterValue(Command(xacro_args), value_type=str)

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
