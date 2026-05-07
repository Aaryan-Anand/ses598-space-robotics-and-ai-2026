import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

from terrain_mapping_drone_control.stack_constants import ROS_DOMAIN_ID


def generate_launch_description():
    """Mission nodes use install-space executables (always matches last colcon build).

    Do not invoke raw ``python3 …/auto_detect_land.py`` from a clone on /mnt/d —
    that bypasses the workspace install and runs stale code.
    """
    env = [SetEnvironmentVariable('ROS_DOMAIN_ID', ROS_DOMAIN_ID)]
    # Do not force ROS_LOCALHOST_ONLY=1: MicroXRCEAgent is not a ROS node and may not
    # participate on loopback-only discovery, which can make /fmu/out/* appear with 0 publishers.

    return LaunchDescription(
        env
        + [
            Node(
                package='terrain_mapping_drone_control',
                executable='aruco_tracker',
                name='aruco_tracker',
                output='screen',
                parameters=[{'use_sim_time': True}],
                arguments=[
                    '--ros-args',
                    '--log-level',
                    'warn',
                ],
            ),
            Node(
                package='terrain_mapping_drone_control',
                executable='auto_detect_land',
                name='cylinder_mission_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ]
    )
