"""Launch pieces shared by the z1_bringup launch files.

The mock, Gazebo-servo, Gazebo-ros2_control and MoveIt paths all start the same
robot: one `robot_state_publisher`, one controller set and (in Gazebo) one
simulator. Everything they have in common lives here so the launch files cannot
drift apart - in particular the list of controllers that gets spawned.
"""

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

CONTROLLER_MANAGER = "/controller_manager"
_HEADLESS = LaunchConfiguration("headless")


def common_args() -> list:
    """Launch arguments accepted by both Gazebo bringup files."""
    return [
        DeclareLaunchArgument(
            "use_gripper",
            default_value="true",
            choices=["true", "false"],
            description="Attach the Unitree gripper to the arm",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="empty.sdf",
            description="Gazebo world file (name resolved by gz-sim or absolute path)",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            choices=["true", "false"],
            description="Run gz-sim server only, without the GUI",
        ),
        DeclareLaunchArgument(
            "gz_verbosity",
            default_value="3",
            description="gz-sim console verbosity (0..4)",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Start RViz2",
        ),
    ]


def gz_sim_includes() -> list:
    """`gz sim`, with the GUI unless headless:=true (two conditional includes)."""
    source = PythonLaunchDescriptionSource(
        [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
    )
    verbosity = LaunchConfiguration("gz_verbosity")
    world = LaunchConfiguration("world")
    return [
        IncludeLaunchDescription(
            source,
            launch_arguments={"gz_args": ["-r -v", verbosity, " ", world]}.items(),
            condition=UnlessCondition(_HEADLESS),
        ),
        IncludeLaunchDescription(
            source,
            launch_arguments={"gz_args": ["-s -r -v", verbosity, " ", world]}.items(),
            condition=IfCondition(_HEADLESS),
        ),
    ]


def robot_state_publisher(robot_description) -> Node:
    """Publishes /robot_description and the TF tree on the Gazebo clock."""
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )


def spawn_model() -> Node:
    """Spawns the model described on /robot_description into the running world."""
    return Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_z1",
        output="screen",
        arguments=["-name", "z1", "-topic", "robot_description"],
    )


def rviz(rviz_config) -> Node:
    """RViz2 on the Gazebo clock, started only with use_rviz:=true."""
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )


def controller_spawner(controller: str, spawner_args: list, use_sim_time: bool = False,
                       condition=None) -> Node:
    """A controller_manager spawner for one controller.

    The ros2_control and the Gazebo bringup paths spawn the same controller set,
    so the list stays in one place (`*_CONTROLLERS` below) instead of being
    repeated in every launch file.
    """
    return Node(
        package="controller_manager",
        executable="spawner",
        name=f"spawner_{controller}",
        output="screen",
        arguments=[controller] + list(spawner_args),
        parameters=[{"use_sim_time": use_sim_time}],
        condition=condition,
    )


# The controller set every bringup path activates (see z1_bringup/config).
ARM_CONTROLLERS = ["joint_state_broadcaster", "joint_trajectory_controller"]
GRIPPER_CONTROLLER = "gripper_controller"
