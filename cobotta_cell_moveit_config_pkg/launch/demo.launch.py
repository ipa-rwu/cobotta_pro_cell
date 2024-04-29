from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():

    current_moveit_pkg = FindPackageShare("cobotta_cell_moveit_config_pkg")
    declared_arguments = []

    ########## moveit related arg ###########
    declared_arguments.append(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )

    declared_arguments.append(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )

    declared_arguments.append(
        DeclareBooleanLaunchArg(
            "use_rviz", default_value=True, description="Whether to use RVIZ or not."
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_moveit_fake_controller",
            default_value="false",
            description="Whether to use moveit fake controller.",
        )
    )

    use_moveit_fake_controller = LaunchConfiguration("use_moveit_fake_controller")

    # ref https://github.com/ros-planning/moveit2/blob/main/moveit_configs_utils/moveit_configs_utils/launches.py
    move_grop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([current_moveit_pkg, "launch", "move_group.launch.py"])
        ),
    )
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [current_moveit_pkg, "launch", "moveit_rviz.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    ########## cobotta related arg ###########
    declared_arguments.append(
        DeclareLaunchArgument(
            name="cobotta_ip_address",
            default_value="192.168.0.2",
            description="The IP address of the robot to connect to.",
        )
    )

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
            "description_package",
            default_value="cobotta_cell_moveit_config_pkg",
            description="Description package with robot URDF/XACRO files. Usually the argument"
            + " is not set, it enables use of a custom description.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="config/cobotta_cell.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "cobotta_namespace",
            default_value="c_210_cobotta_pro_",
            description="Prefix of the joint names, useful for"
            + " multi-robot setup. If changed than also joint names in the controllers'"
            + " configuration have to be updated.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "cobotta_controller_pkg",
            default_value="denso_robot_control",
            description="pkg with robot controller config",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "cobotta_controller",
            default_value="denso_joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "cobotta_controllers_file",
            default_value="config/denso_robot_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Using plugin mock_components/GenericSystem; Start robot with fake hardware mirroring command to its states.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "cobotta_joint_state_broadcaster",
            default_value="denso_joint_state_broadcaster",
            description="the key defined in the controller",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "cobotta_model",
            default_value="cvrb0609",
            description="Type/series of used denso robot.",
        )
    )

    cobotta_model = LaunchConfiguration("cobotta_model")
    cobotta_ip_address = LaunchConfiguration("cobotta_ip_address")
    send_format = LaunchConfiguration("send_format")
    recv_format = LaunchConfiguration("recv_format")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    cobotta_namespace = LaunchConfiguration("cobotta_namespace")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    cobotta_controllers_file = LaunchConfiguration("cobotta_controllers_file")
    cobotta_controller_pkg = LaunchConfiguration("cobotta_controller_pkg")
    cobotta_controller = LaunchConfiguration("cobotta_controller")
    cobotta_joint_state_broadcaster = LaunchConfiguration(
        "cobotta_joint_state_broadcaster"
    )

    ################### Real Robot Controller ############################
    cobotta_pro_control_launch = IncludeLaunchDescription(
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
            "model": cobotta_model,
            "send_format": send_format,
            "recv_format": recv_format,
            "ip_address": cobotta_ip_address,
            "description_package": description_package,
            "description_file": description_file,
            "namespace": cobotta_namespace,
            "robot_controller_pkg": cobotta_controller_pkg,
            "robot_controller": cobotta_controller,
            "controllers_file": cobotta_controllers_file,
            "use_mock_hardware": use_mock_hardware,
            "joint_state_broadcaster": cobotta_joint_state_broadcaster,
            "use_moveit_fake_controller": use_moveit_fake_controller,
            "sim": "false",
            "launch_rviz": "false",
        }.items(),
    )

    #####################################################################

    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare(description_package), description_file]
                ),
                " ",
                "ip_address:=",
                cobotta_ip_address,
                " ",
                "model:=",
                cobotta_model,
                " ",
                "send_format:=",
                send_format,
                " ",
                "recv_format:=",
                recv_format,
                " ",
                "namespace:=",
                cobotta_namespace,
                " ",
                "verbose:=false",
                " ",
                "sim:=false",
                " ",
                "use_mock_hardware:=",
                use_mock_hardware,
                " ",
            ]
        ),
        value_type=str,
    )

    robot_description = {"robot_description": robot_description_content}

    # Fake joint driver; ref https://github.com/ros-planning/moveit2/blob/main/moveit_configs_utils/moveit_configs_utils/launches.py
    # Fake joint driver
    moveit_contoller_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([current_moveit_pkg, "config/ros2_controllers.yaml"]),
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        condition=IfCondition(LaunchConfiguration("use_moveit_fake_controller")),
    )
    swap_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [current_moveit_pkg, "launch", "spawn_controllers.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_moveit_fake_controller")),
    )
    robot_state_publlisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([current_moveit_pkg, "launch", "rsp.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("use_moveit_fake_controller")),
    )

    nodes_to_start = [
        cobotta_pro_control_launch,
        moveit_contoller_node,
        swap_controller_launch,
        robot_state_publlisher,
        move_grop_launch,
        moveit_rviz_launch,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
