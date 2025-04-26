import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    rsp_argument = DeclareLaunchArgument('rsp', default_value='true',
                          description='Run robot state publisher node.')

    pkg = get_package_share_directory('carrie_description')

    # Obtain urdf from xacro files.
    xacro_file = os.path.join(pkg, 'urdf', 'carrie.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    params = {'robot_description': robot_description,
              'publish_frequency': 30.0}

    # Robot state publisher
    rsp = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace='',
                output='both',
                parameters=[params],
                condition=IfCondition(LaunchConfiguration('rsp'))
    )

    return LaunchDescription([
        rsp_argument,
        rsp,
    ])
