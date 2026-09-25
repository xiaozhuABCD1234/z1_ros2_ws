#!/usr/bin/env python3
"""Bring up the Z1 in Gazebo Sim (gz-sim 8) with bridged joint interfaces.

This path is deliberately independent from ros2_control: the joints are driven
by the gz JointPositionController servos declared in z1_gazebo.xacro and the
bridged ROS interfaces are plain std_msgs/Float64 position commands.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from z1_bringup.launch_parts import (
    common_args,
    gz_sim_includes,
    robot_state_publisher,
    rviz,
    spawn_model,
)


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = FindPackageShare("z1_bringup")
    pkg_description = FindPackageShare("z1_description")

    xacro_file = PathJoinSubstitution([pkg_description, "urdf", "z1.urdf.xacro"])
    rviz_config = PathJoinSubstitution([pkg_description, "rviz", "z1_display.rviz"])
    bridge_config = PathJoinSubstitution([pkg_bringup, "config", "ros_gz_bridge.yaml"])

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " use_gripper:=",
                LaunchConfiguration("use_gripper"),
                " hardware_plugin:=mock_components/GenericSystem",
                " use_gazebo:=true",
            ]
        ),
        value_type=str,
    )

    ros_gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_bridge"), "/launch/ros_gz_bridge.launch.py"]
        ),
        launch_arguments={
            "bridge_name": "ros_gz_bridge",
            "config_file": bridge_config,
        }.items(),
    )

    nodes = (
        gz_sim_includes()
        + [
            robot_state_publisher(robot_description),
            spawn_model(),
            ros_gz_bridge,
            rviz(rviz_config),
        ]
    )

    return LaunchDescription(common_args() + nodes)
