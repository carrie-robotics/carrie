#!/usr/bin/env python3
import os
import xacro

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('carrie_description')
    xacro_file = os.path.join(pkg, 'urdf', 'carrie.urdf.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'view_robot.rviz')

    robot_description = xacro.process_file(xacro_file).toxml()

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        jsp_node,
        rsp_node,
        rviz_node
    ])
