#!/usr/bin/env python3

import os
import shlex
import shutil

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from terrain_mapping_drone_control.stack_constants import ROS_DOMAIN_ID, UXRCE_UDP_PORT


def _micro_xrce_agent_executable():
    """Resolve Micro XRCE DDS agent binary.

    Prefer a non-snap build (snap + Humble Fast DDS often yields uXRCE
    ``create entities failed`` / ping timeouts). Override with MICRO_XRCE_DDS_AGENT.
    """
    override = (os.environ.get('MICRO_XRCE_DDS_AGENT') or '').strip()
    if override and os.path.isfile(override):
        return override
    for path in (
        '/opt/ros/humble/bin/micro-xrce-dds-agent',
        '/opt/ros/humble/bin/MicroXRCEAgent',
        '/usr/local/bin/micro-xrce-dds-agent',
        '/usr/local/bin/MicroXRCEAgent',
    ):
        if os.path.isfile(path):
            return path
    found = []
    for name in ('micro-xrce-dds-agent', 'MicroXRCEAgent'):
        p = shutil.which(name)
        if p:
            found.append(p)
    non_snap = [p for p in found if '/snap/' not in p]
    if non_snap:
        return non_snap[0]
    if found:
        return found[0]
    snap = '/snap/bin/micro-xrce-dds-agent'
    return snap if os.path.isfile(snap) else None


def _micro_xrce_ld_library_path_export():
    """Bash `export LD_LIBRARY_PATH=...` so /usr/local/bin/MicroXRCEAgent finds libmicroxrcedds_agent.so.

    `sudo make install` often copies only the binary; libraries stay in the build tree unless you
    run `sudo ldconfig`. We prepend usual build locations (see WSL smoke test in repo history).
    """
    parts = []
    extra = (os.environ.get('MICRO_XRCE_DDS_LIB_DIR') or '').strip()
    if extra and os.path.isdir(extra):
        parts.append(extra)
    parts.append('/usr/local/lib')
    home = os.path.expanduser('~')
    for sub in (
        os.path.join(home, 'Micro-XRCE-DDS-Agent', 'build'),
        os.path.join(home, 'ros2_ws', 'src', 'terrain_mapping_drone_control', 'scripts', 'Micro-XRCE-DDS-Agent', 'build'),
    ):
        so = os.path.join(sub, 'libmicroxrcedds_agent.so')
        so_ver = os.path.join(sub, 'libmicroxrcedds_agent.so.3.0')
        if os.path.isdir(sub) and (os.path.isfile(so) or os.path.isfile(so_ver)):
            parts.append(sub)
    # Dedupe, preserve order
    seen, ordered = set(), []
    for p in parts:
        if p not in seen:
            seen.add(p)
            ordered.append(p)
    joined = ':'.join(ordered)
    return f'export LD_LIBRARY_PATH={joined}:${{LD_LIBRARY_PATH:-}}; '


def generate_launch_description():
    """Generate launch description for cylinder landing mission."""

    # WSLg exposes Wayland by default; Gazebo Harmonic + Qt6 often show no window without X11.
    wsl_gui_env = []
    if os.environ.get('WSL_DISTRO_NAME'):
        wsl_gui_env = [
            SetEnvironmentVariable('DISPLAY', os.environ.get('DISPLAY') or ':0'),
            SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
            SetEnvironmentVariable('GDK_BACKEND', 'x11'),
            UnsetEnvironmentVariable('WAYLAND_DISPLAY'),
        ]

    # Get the package share directory
    pkg_share = get_package_share_directory('terrain_mapping_drone_control')
        
    # Set Gazebo model and resource paths
    gz_model_path = os.path.join(pkg_share, 'models')

    # # Set initial drone pose
    os.environ['PX4_GZ_MODEL_POSE'] = "0,0,0.1,0,0,0"
    
    # Add launch argument for PX4-Autopilot path
    px4_autopilot_path = LaunchConfiguration('px4_autopilot_path')
    start_agent = LaunchConfiguration('start_micro_xrce_agent')

    agent_exe = _micro_xrce_agent_executable()
    # Dedicated agent on UXRCE_UDP_PORT + ROS_DOMAIN_ID so we do not share DDS domain 0 with snap (8888).
    micro_xrce_agent = None
    if agent_exe:
        warn_snap = ''
        if '/snap/' in agent_exe:
            warn_snap = (
                "echo '[cylinder_landing] WARNING: using snap micro-xrce-dds-agent; if uXRCE "
                "create entities failed / ping loops, install a local agent (see README) or set "
                "MICRO_XRCE_DDS_AGENT.' 1>&2; "
            )
        # Do not run fuser here: on some setups it can SIGKILL the agent mid-flight if the
        # wrapper is re-run or the port is shared; clear the port from mission scripts instead.
        agent_bash = (
            f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID}; "
            f"{_micro_xrce_ld_library_path_export()}"
            f"{warn_snap}"
            f"exec {shlex.quote(agent_exe)} udp4 -p {UXRCE_UDP_PORT}"
        )
        micro_xrce_agent = ExecuteProcess(
            cmd=['bash', '-c', agent_bash],
            output='screen',
            condition=IfCondition(start_agent),
        )

    # PX4 must use the same ROS_DOMAIN_ID and UDP port as the Micro XRCE agent above.
    px4_sitl = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            (
                f'export ROS_DOMAIN_ID={ROS_DOMAIN_ID} PX4_UXRCE_DDS_PORT={UXRCE_UDP_PORT}; '
                'exec make px4_sitl gz_x500_depth_mono'
            ),
        ],
        cwd=px4_autopilot_path,
        output='screen'
    )
    
    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder', 'model.sdf'),
            '-name', 'cylinder_front',
            '-x', '5',     # 5 meters in front of the drone
            '-y', '0',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the second cylinder (behind, 7m height)
    spawn_cylinder_back = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_back',
            '-x', '-5',    # 5 meters behind the drone
            '-y', '0',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Bridge node for camera and odometry
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=[
            # Front RGB Camera
            '/rgb_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            
            # Front Depth Camera
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            # '/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            
            # Down Mono Camera
            '/mono_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mono_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            
            # Clock and Odometry
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/model/x500_depth_mono_0/odometry_with_covariance@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance',
        ],
        remappings=[
            # Front RGB Camera remappings
            ('/rgb_camera', '/drone/front_rgb'),
            ('/rgb_camera/camera_info', '/drone/front_rgb/camera_info'),
            
            # Front Depth Camera remappings
            ('/depth_camera', '/drone/front_depth'),
            # ('/depth_camera/depth_image', '/drone/front_depth/depth'),
            ('/depth_camera/points', '/drone/front_depth/points'),
            ('/camera_info', '/drone/front_depth/camera_info'),
            
            # Down Mono Camera remappings
            ('/mono_camera', '/drone/down_mono'),
            ('/mono_camera/camera_info', '/drone/down_mono/camera_info'),
            
            # Gazebo odometry must NOT use /fmu/out/vehicle_odometry (PX4 uXRCE owns that as px4_msgs).
            ('/model/x500_depth_mono_0/odometry_with_covariance', '/drone/gz_odometry'),
        ],
        output='screen'
    )

    # PX4 @ +1.5 s; cylinders after sim is up; ros_gz bridge last so heavy DDS traffic does not
    # race uXRCE entity creation (reduces create-entities / no-ping loops on WSL).
    delayed_px4 = TimerAction(period=1.5, actions=[px4_sitl])
    delayed_spawn_front = TimerAction(period=5.0, actions=[spawn_cylinder_front])
    delayed_spawn_back = TimerAction(period=5.5, actions=[spawn_cylinder_back])
    delayed_bridge = TimerAction(period=10.0, actions=[bridge])

    core = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'px4_autopilot_path',
            default_value=os.environ.get('HOME', '/home/' + os.environ.get('USER', 'user')) + '/PX4-Autopilot',
            description='Path to PX4-Autopilot directory'),
        DeclareLaunchArgument(
            'start_micro_xrce_agent',
            default_value='true',
            description=(
                f'Start Micro XRCE DDS Agent on UDP {UXRCE_UDP_PORT} with ROS_DOMAIN_ID={ROS_DOMAIN_ID} '
                '(isolates from snap on 8888/domain 0). Set false only if you manage the agent yourself.'
            )),
    ]
    if micro_xrce_agent is not None:
        core.append(micro_xrce_agent)
    core += [
        delayed_px4,
        delayed_spawn_front,
        delayed_spawn_back,
        delayed_bridge,
    ]

    dds_env = [SetEnvironmentVariable('ROS_DOMAIN_ID', ROS_DOMAIN_ID)]
    # Do not force ROS_LOCALHOST_ONLY=1: MicroXRCEAgent is not a ROS node and may not
    # participate on loopback-only discovery, which can make /fmu/out/* appear with 0 publishers.

    return LaunchDescription(dds_env + wsl_gui_env + core)