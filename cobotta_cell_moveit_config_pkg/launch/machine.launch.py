def generate_launch_description():
    moveit_fake_controller = DeclareBooleanLaunchArg(
        "use_moveit_fake_controller",
        default_value=False,
        description="Whether to use moveit fake controller.",
    )
    rviz_config = DeclareBooleanLaunchArg(
        "use_rviz", default_value=True, description="Whether to use RVIZ or not."
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "ip_address:=",
            ip_address,
            " ",
            "model:=",
            denso_robot_model,
            " ",
            "send_format:=",
            send_format,
            " ",
            "recv_format:=",
            recv_format,
            " ",
            "namespace:=",
            namespace,
            " ",
            "verbose:=",
            verbose,
            " ",
            "sim:=",
            sim,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

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

    ################### Real Robot Controller ############################
    denso_robot_control_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "use_fake_hardware": "false",
            "initial_joint_controller": "joint_trajectory_controller",
            "activate_joint_controller": "true",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "description_package": "ur5e_cell_description",
            "description_file": "workcell.urdf.xacro",
            "launch_rviz": "false",
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("use_moveit_fake_controller")),
    )

    #####################################################################

    # Fake joint driver; ref https://github.com/ros-planning/moveit2/blob/main/moveit_configs_utils/moveit_configs_utils/launches.py
    moveit_contoller_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
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
