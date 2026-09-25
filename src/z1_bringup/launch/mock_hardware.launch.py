#!/usr/bin/env python3
"""Z1 ros2_control with mock hardware + RViz2.

The mock hardware (mock_components/GenericSystem) acts as a perfect servo, so
this exercises the controller stack (joint_state_broadcaster,
joint_trajectory_controller, gripper action) without Gazebo or the arm.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    rviz_default_config = PathJoinSubstitution(
        [FindPackageShare("z1_description"), "rviz", "z1_display.rviz"]
    )

    declared_args = [
        DeclareLaunchArgument(
            "use_gripper",
            default_value="true",
            choices=["true", "false"],
            description="Attach the Unitree gripper to the arm",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Start RViz2",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_default_config,
            description="RViz2 configuration file",
        ),
    ]

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("z1_bringup"), "/launch/control.launch.py"]
        ),
        launch_arguments={"use_gripper": LaunchConfiguration("use_gripper")}.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(declared_args + [control, rviz])
