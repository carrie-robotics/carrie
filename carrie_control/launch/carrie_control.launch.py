import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    controller_params_file = os.path.join(get_package_share_directory("carrie_control"),'config','carrie_controllers.yaml')

    # We need the robot description to be passed to the controller_manager
    # So it can check the ros2_control parameters.
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)},
                    controller_params_file],
        remappings=[
            ('/diff_controller/cmd_vel', 'cmd_vel_stamped'),
            ('/diff_controller/cmd_vel_out', 'cmd_vel_out'),
            ('/diff_controller/odom', 'odom'),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of diff_drive_controller_spawner after `joint_state_broadcaster`
    delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    relay_node = Node(
        package="topic_tools",
        executable="relay_field",
        name="cmd_vel_relay",
        arguments=["cmd_vel", "cmd_vel_stamped", "geometry_msgs/TwistStamped", "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: m}", "--wait-for-start"],
    )

    nodes = [
        control_node,
        joint_state_broadcaster_spawner,
        delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner,
        relay_node,
    ]

    return LaunchDescription(nodes)
