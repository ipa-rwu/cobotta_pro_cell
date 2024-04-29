from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg

from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = []

    # Denso specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "model",
            default_value="cvrb0609",
            description="Type/series of used denso robot.",
        )
    )
    # TODO: shall we let the user to only select from a list of robots ??
    # choices=['cobotta', 'vs060', 'vs087', 'cvrb0609']))
    declared_arguments.append(
        DeclareLaunchArgument(
            "send_format",
            default_value="32",
            description="Data format for sending commands to the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "recv_format",
            default_value="34",
            description="Data format for receiving robot status.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bcap_slave_control_cycle_msec",
            default_value="8.0",
            description="Control frequency.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ip_address",
            default_value="192.168.0.2",
            description="IP address by which the robot can be reached.",
        )
    )
    # Configuration arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="cobotta_cell_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
                is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="cobotta_cell.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "denso_robot_namespace",
            default_value="c_210_cobotta_pro_",  # 'denso',
            description="Prefix of the joint names, useful for \
                multi-robot setup. If changed than also joint names in the controllers' \
                configuration have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="config/ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller_pkg",
            default_value="cobotta_cell_moveit_config",
            description="Package with robot controller config",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="c_210_cobotta_pro_controller",
            description="Robot controller to start.",
        )
    )
    # Execution arguments (Rviz and Gazebo)
    # TODO: shall we give the user the choice not to load the rviz graphical environment ??
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "verbose",
            default_value="false",
            description="Print out additional debug information.",
        )
    )

    ################### Real Robot Controller ############################
    denso_robot_control_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("denso_robot_control"),
                    "launch",
                    "denso_robot_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "send_format": LaunchConfiguration("send_format"),
            "recv_format": LaunchConfiguration("recv_format"),
            "ip_address": LaunchConfiguration("ip_address"),
            "description_package": LaunchConfiguration("description_package"),
            "description_file": LaunchConfiguration("description_file"),
            "namespace": LaunchConfiguration("denso_robot_namespace"),
            "controllers_file": LaunchConfiguration("controllers_file"),
            "robot_controller_pkg": LaunchConfiguration("robot_controller_pkg"),
            "robot_controller": LaunchConfiguration("robot_controller"),
            "sim": LaunchConfiguration("sim"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "model": LaunchConfiguration("model"),
        }.items(),
    )

    nodes_to_start = [
        denso_robot_control_launch,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
