#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap

from nav2_common.launch import ParseMultiRobotPose

from carrie_gz.launch_tools.substitutions import TextJoin


def generate_launch_description():
    pkg_carrie_gz = get_package_share_directory('carrie_gz')

    ros_bridge_arg = DeclareLaunchArgument(
        'ros_bridge', default_value='True', description='Run ROS bridge node.')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='True', description='Start RViz.')
    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='empty.sdf', description='Name of the world to load.')
    robots_arg = DeclareLaunchArgument(
        'robots', default_value="carrie={x: 0., y: 0., z: 0.1, yaw: 0.};",
        description='Robots to spawn, multiple robots can be stated separated by a ; ')
    gui_config_arg = DeclareLaunchArgument(
        'gui_config',
        default_value='default.config',
        description='Name of the gui configuration file to load.')

    # Variables of launch file.
    rviz = LaunchConfiguration('rviz')
    ros_bridge = LaunchConfiguration('ros_bridge')
    world_name = LaunchConfiguration('world_name')
    gui_config = LaunchConfiguration('gui_config')
    gui_config_path = PathJoinSubstitution([pkg_carrie_gz, 'config_gui', gui_config])

    # Obtains world path.
    world_path = PathJoinSubstitution([pkg_carrie_gz, 'worlds', world_name])
    log_world_path = LogInfo(msg=TextJoin(substitutions=["World path: ", world_path]))
    # Gazebo arguments.
    gz_args = TextJoin(
        substitutions=[
            world_path,
            TextJoin(substitutions=["--gui-config", gui_config_path], separator=' '),
        ],
        separator=' ',
    )
    # Launches the base group: Gazebo sim and ROS bridge for generic Gazebo stuff.
    base_group = GroupAction(
        scoped=True, forwarding=False,
        launch_configurations={
            'ros_bridge': ros_bridge,
            'world_name': world_name,
            'gui_config': gui_config,
        },
        actions=[
            # Gazebo Sim
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
                ),
                launch_arguments={'gz_args': gz_args}.items(),
            ),
            # ROS Bridge for generic Gazebo stuff
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen',
                namespace='carrie_gz_sim',
                condition=IfCondition(ros_bridge),
            ),
        ]
    )

    robots_list = ParseMultiRobotPose('robots').value()
    # When no robots are specified, spawn a single robot at the origin.
    # The default value isn't getting parsed correctly because ParseMultiRobotPose checks sys.args
    # instead of using launch argument.
    log_robots_by_user = LogInfo(msg="Robots provided by user.")
    if (robots_list == {}):
        log_robots_by_user = LogInfo(msg="No robots provided, using default:")
        robots_list = {"andino": {"x": 0., "y": 0., "z": 0.1, "yaw": 0.}}
    log_number_robots = LogInfo(msg="Robots to spawn: " + str(robots_list))
    spawn_robots_group = []
    more_than_one_robot = PythonExpression([TextSubstitution(text=str(len(robots_list.keys()))), ' > 1'])
    one_robot = PythonExpression([TextSubstitution(text=str(len(robots_list.keys()))), ' == 1'])
    for robot_name in robots_list:
        init_pose = robots_list[robot_name]
        # As it is scoped and not forwarding, the launch configuration in this context gets cleared.
        robots_group = GroupAction(
            scoped=True, forwarding=False,
            launch_configurations={
                'rviz': rviz,
                'ros_bridge': ros_bridge,
            },
            actions=[
                LogInfo(msg="Group for robot: " + robot_name),
                PushRosNamespace(
                    condition=IfCondition(more_than_one_robot),
                    namespace=robot_name),
                # Spawn the robot and the Robot State Publisher node.
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_carrie_gz, 'launch', 'include', 'spawn_robot.launch.py')
                    ),
                    launch_arguments={
                        'entity': robot_name,
                        'initial_pose_x': str(init_pose['x']),
                        'initial_pose_y': str(init_pose['y']),
                        'initial_pose_z': str(init_pose['z']),
                        'initial_pose_yaw': str(init_pose['yaw']),
                        'robot_description_topic': 'robot_description',
                        'use_sim_time': 'true',
                    }.items(),
                ),
                # RViz
                Node(
                    condition=IfCondition(PythonExpression([rviz])),
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', os.path.join(pkg_carrie_gz, 'rviz', 'carrie_gz.rviz')],
                    parameters=[{'use_sim_time': True}],
                    remappings=[
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                    ],
                ),
                # Run ros_gz bridge
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_carrie_gz, 'launch', 'include', 'gz_ros_bridge.launch.py')
                    ),
                    launch_arguments={
                        'entity': robot_name,
                    }.items(),
                    condition=IfCondition(LaunchConfiguration('ros_bridge')),
                ),
            ]
        )

        spawn_robots_group.append(robots_group)

    ld = LaunchDescription()
    ld.add_action(log_robots_by_user)
    ld.add_action(log_number_robots)
    ld.add_action(ros_bridge_arg)
    ld.add_action(rviz_arg)
    ld.add_action(world_name_arg)
    ld.add_action(robots_arg)
    ld.add_action(gui_config_arg)
    ld.add_action(log_world_path)
    ld.add_action(base_group)
    for group in spawn_robots_group:
        ld.add_action(group)
    return ld
