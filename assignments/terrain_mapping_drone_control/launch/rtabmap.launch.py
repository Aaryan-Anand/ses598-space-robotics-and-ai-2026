#!/usr/bin/env python3
"""RTAB-Map + camera TF for PX4/Gazebo bridge topics (use with cylinder_landing.launch.py)."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, UnsetEnvironmentVariable
from launch_ros.actions import Node

from terrain_mapping_drone_control.stack_constants import ROS_DOMAIN_ID

ODOM_FRAME = 'odom'
# Must match camera topics' header.frame_id from ros_gz_bridge.
CAM_FRAME = 'OakD-Lite-Modify/base_link'


def generate_launch_description():
    wsl_gui = []
    if os.environ.get('WSL_DISTRO_NAME'):
        wsl_gui = [
            SetEnvironmentVariable('DISPLAY', os.environ.get('DISPLAY') or ':0'),
            SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
            SetEnvironmentVariable('GDK_BACKEND', 'x11'),
            UnsetEnvironmentVariable('WAYLAND_DISPLAY'),
        ]

    odom_tf = Node(
        package='terrain_mapping_drone_control',
        executable='odom_to_camera_tf',
        name='odom_to_camera_tf',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # On Humble, the `rtabmap` executable lives under `rtabmap_slam` (not `rtabmap_ros`).
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'frame_id': CAM_FRAME,
                'odom_frame_id': ODOM_FRAME,
                'subscribe_depth': True,
                'subscribe_rgb': True,
                # rtabmap_slam (Humble) doesn't reliably create an odom subscription; run RGB-D SLAM
                # without external odometry and rely on its internal visual odom.
                'subscribe_odom': False,
                'approx_sync': True,
                'queue_size': 30,
                # ros_gz_bridge publishes camera topics as RELIABLE by default.
                # Force RTAB-Map subscriptions to RELIABLE so it actually receives images/camera_info.
                # (With BEST_EFFORT subscribers, Humble often shows "Did not receive data".)
                'qos_image': 1,
                'qos_camera_info': 1,
                # Output maps periodically even when graph optimization doesn't add many nodes yet.
                'Rtabmap/PublishMap': 'true',
                'Rtabmap/DetectionRate': '1.0',
                'Vis/MinInliers': '12',
                'RGBD/NeighborLinkRefining': 'True',
                'Reg/Strategy': '1',
            }
        ],
        remappings=[
            ('rgb/image', '/drone/front_rgb'),
            # This bridge publishes only one camera_info reliably; use the depth intrinsics for both.
            ('rgb/camera_info', '/drone/front_depth/camera_info'),
            ('depth/image', '/drone/front_depth'),
            ('depth/camera_info', '/drone/front_depth/camera_info'),
        ],
    )

    dds_env = [SetEnvironmentVariable('ROS_DOMAIN_ID', ROS_DOMAIN_ID)]
    # Do not force ROS_LOCALHOST_ONLY=1 (see mission.launch.py note).

    return LaunchDescription(
        dds_env
        + wsl_gui
        + [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation clock (always true for this SITL stack)',
            ),
            odom_tf,
            rtabmap,
        ]
    )
