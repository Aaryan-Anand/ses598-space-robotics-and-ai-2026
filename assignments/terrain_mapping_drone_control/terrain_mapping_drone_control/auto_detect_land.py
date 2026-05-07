#!/usr/bin/env python3

import math
import os
import sys
import time
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)

from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    TrajectorySetpoint,
    BatteryStatus,
    VehicleLocalPosition,
)
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
import numpy as np

# For synchronized subscription of RGB + Depth
from message_filters import ApproximateTimeSynchronizer, Subscriber


class CylinderMission(Node):
    """PX4 offboard mission with RGB-D cues and ArUco landing."""

    _TELEM_STATES = frozenset({
        'ARM_TAKEOFF',
        'CIRCLE',
        'SERVO',
        'HOVER',
        'ARUCO_HOVER',
        'ARUCO_SELECT',
        'ARUCO_MOVE',
        'ARUCO_LAND',
    })

    def __init__(self):
        super().__init__('cylinder_mission_node')

        # ---------------------------------------------
        # PX4 / Offboard QoS
        # ---------------------------------------------
        # Publishers to /fmu/in/*: TRANSIENT_LOCAL matches common px4_ros_com / MicroXRCE examples.
        qos_fmu_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Subscribers to /fmu/out/*: PX4 publishes volatile best-effort streams; TRANSIENT_LOCAL subs
        # often do not match → no samples (mission stuck in WAIT for vehicle_local_position).

        # ---------------------------------------------
        # Publishers
        # ---------------------------------------------
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_fmu_in
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_fmu_in
        )
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_fmu_in
        )

        # ---------------------------------------------
        # Subscribers
        # ---------------------------------------------
        # Drone odometry from Gazebo (nav_msgs). Kept off /fmu/out/vehicle_odometry so PX4 uXRCE
        # can own that topic as px4_msgs/VehicleOdometry without a DDS type clash.
        # Must match ros_gz_bridge (BEST_EFFORT): RELIABLE subs often get zero samples → mission stuck.
        self.vehicle_odometry_sub = self.create_subscription(
            Odometry, '/drone/gz_odometry', self.odom_cb, qos_profile_sensor_data
        )
        # PX4 NED local position (authoritative for offboard; Gazebo odom often stays zero here).
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_cb,
            qos_profile_sensor_data,
        )

        # Camera intrinsics: bridge may publish depth and RGB at different rates; subscribe to both.
        self.caminfo_depth_sub = self.create_subscription(
            CameraInfo,
            '/drone/front_depth/camera_info',
            self.caminfo_callback,
            qos_profile_sensor_data,
        )
        self.caminfo_rgb_sub = self.create_subscription(
            CameraInfo,
            '/drone/front_rgb/camera_info',
            self.caminfo_callback,
            qos_profile_sensor_data,
        )

        # Approx time sync for RGB + Depth (same QoS as Gazebo bridge)
        self.rgb_sub = Subscriber(
            self, Image, '/drone/front_rgb', qos_profile=qos_profile_sensor_data
        )
        self.depth_sub = Subscriber(
            self, Image, '/drone/front_depth', qos_profile=qos_profile_sensor_data
        )
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.image_callback)

        # ---------------------------------------------
        # Internal State Machine
        # ---------------------------------------------
        # WAIT_INTRINSICS -> ARM_TAKEOFF -> CIRCLE -> SERVO -> HOVER
        # -> LAND -> DISARM -> COMPLETE -> DONE
        self.takeoff_stage = 0  # 0 = vertical, 1 = move to circle start

        self.state = "WAIT_INTRINSICS"
        self.offboard_setpoint_counter = 0

        # Timer for controlling flight logic
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Current drone position [x,y,z] in PX4 local NED (from /fmu/out/vehicle_local_position).
        self.position = [0.0, 0.0, 0.0]
        self._vlp_heading = 0.0
        self._vlp_received = False
        self._vlp_logged = False
        self.bridge = CvBridge()

        # ---------------------------------------------
        # Camera intrinsics
        # ---------------------------------------------
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # ---------------------------------------------
        # Circle flight parameters
        # ---------------------------------------------
        self.circle_radius = 15.0
        self.altitude = -5.0
        self.circle_speed = -0.02  # radians step per iteration
        self.theta = 0.0

        # ---------------------------------------------
        # Cylinder detection and measurement
        # ---------------------------------------------
        self.measured_cylinders = []
        self.points_buffer = []
        self.sample_threshold = 10  # frames to accumulate for stable measurement
        self.desired_distance = 15.0
        # Tight SERVO tolerances + conservative vision reduce false CIRCLE→SERVO trips that
        # feed bogus depth Z into huge horizontal setpoints (runaway).
        self.distance_tolerance = 0.35
        self.hover_start_time = None
        self.servo_start_time = None
        self.min_pixel_area = 5000
        # Only treat as a cylinder cue if depth is in a plausible range (meters in camera frame).
        self.detection_depth_min = 4.0
        self.detection_depth_max = 22.0

        # ---------------------------------------------
        # Detection cooldown control
        # ---------------------------------------------
        # We skip detection for a few seconds after measuring / rejecting a detection
        self.detection_cooldown_until = 0.0
        # Height comparison tolerance (m) when distinguishing cylinders / tallest
        self.cylinder_dim_tolerance = 0.45

        # ---------------------------------------------
        # Land on tallest cylinder
        # ---------------------------------------------
        # For ArUco logic
        self.markers = {}

        # ArUco marker pose subscriber (string topic)
        self.marker_pose_sub = self.create_subscription(
            String, '/aruco/marker_pose', self.aruco_cb, 10
        )
        self.aruco_hover_start_time = None
        # Hold horizontal position while ascending/descending for ArUco (NED local frame).
        self._aruco_hover_xy = None
        self._aruco_hover_enter_time = None
        self._aruco_hover_hold_sec = 5.0
        self._aruco_select_enter_time = None
        self.land_marker_id = None
        self._aruco_move_start_time = None
        self._land_command_time = None
        self._aruco_land_disarm_done = False

        # ---------------------------------------------
        # One-shot offboard + arm after a few control cycles (avoid counter==5 race with WAIT_INTRINSICS)
        self._initial_arm_done = False
        self._last_wait_status_log = 0.0
        # Whole-mission pose samples for plots / grading (not only ARM_TAKEOFF).
        self._last_pose_telemetry_log = None
        self._pose_telemetry_interval_sec = 2.0
        self._odom_logged = False

        # Logging mission details
        # ---------------------------------------------
        # Mission timing and energy tracking
        self.start_time = None
        self.battery_percent = None
        self.initial_battery = None

        # For mission battery tracking
        self.battery_at_mission_start = None
        self.battery_at_mission_end = None

        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.battery_cb,
            qos_profile_sensor_data,
        )

    # ---------------------------------------------
    # Battery logging
    # ---------------------------------------------
    def battery_cb(self, msg):
        if not math.isnan(msg.volt_based_soc_estimate):
            # Keep an up-to-date snapshot of battery percentage
            self.battery_percent = msg.volt_based_soc_estimate

    # ---------------------------------------------
    # Callback: Vehicle Odometry
    # ---------------------------------------------
    def odom_cb(self, msg):
        if not self._odom_logged:
            self._odom_logged = True
            self.get_logger().info('Receiving /drone/gz_odometry (for debug; mission uses PX4 local position).')

    def vehicle_local_position_cb(self, msg: VehicleLocalPosition):
        self.position = [float(msg.x), float(msg.y), float(msg.z)]
        h = getattr(msg, 'heading', float('nan'))
        if not math.isnan(h):
            self._vlp_heading = float(h)
        self._vlp_received = True
        if not self._vlp_logged:
            self._vlp_logged = True
            self.get_logger().info(
                f'Receiving /fmu/out/vehicle_local_position NED z={self.position[2]:.2f} '
                '(used for offboard setpoints and progress).'
            )

    # ---------------------------------------------
    # Callback: Camera Info (intrinsics)
    # ---------------------------------------------
    def caminfo_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        self.get_logger().info('Camera intrinsics received.')
        if self.caminfo_depth_sub is not None:
            self.destroy_subscription(self.caminfo_depth_sub)
            self.caminfo_depth_sub = None
        if self.caminfo_rgb_sub is not None:
            self.destroy_subscription(self.caminfo_rgb_sub)
            self.caminfo_rgb_sub = None

    # ---------------------------------------------
    # Callback: Synchronized Image + Depth
    # ---------------------------------------------
    def image_callback(self, rgb_msg, depth_msg):
        # Skip detection during cooldown
        if time.time() < self.detection_cooldown_until:
            return

        # If intrinsics are not known yet, skip
        if self.fx is None or self.fy is None:
            return

        # Convert ROS → OpenCV
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough').astype(np.float32)
        depth[depth == 0] = np.nan

        # HSV color segmentation + depth gating for cylinder-shaped cues
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([0, 0, 110])
        upper_hsv = np.array([180, 40, 180])
        color_mask = cv2.inRange(hsv, lower_hsv, upper_hsv) > 0

        # Depth threshold
        depth_mask = np.logical_and(depth > 1.0, depth < 30.0)
        object_mask = np.logical_and(depth_mask, color_mask)

        # Morphological close to reduce noise
        object_mask = cv2.morphologyEx(
            object_mask.astype(np.uint8),
            cv2.MORPH_CLOSE,
            np.ones((5, 5), np.uint8)
        )

        # Contour detection
        contours, _ = cv2.findContours(object_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered = [c for c in contours if cv2.contourArea(c) > self.min_pixel_area]


        # Visualization overlay
        overlay = rgb.copy()

        if len(filtered) > 0:
            # Sort by largest area
            filtered.sort(key=cv2.contourArea, reverse=True)
            contour = filtered[0]
            x, y, w, h = cv2.boundingRect(contour)
            roi = depth[y:y + h, x:x + w]
            roi = roi[np.isfinite(roi)]

            if roi.size > 0:
                # Median depth in bounding box
                Z = float(np.median(roi))
                width_m = (w * Z) / self.fx
                height_m = (h * Z) / self.fy

                self.points_buffer.append((width_m, height_m, Z))

                # Debug bounding box
                cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(
                    overlay,
                    f"{width_m:.2f}m x {height_m:.2f}m",
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1
                )

                # Transition to SERVO only with plausible depth (avoids sky/ground false positives).
                if (
                    self.state == "CIRCLE"
                    and self.detection_depth_min < Z < self.detection_depth_max
                ):
                    self.get_logger().info(
                        f"Detected potential cylinder (Z={Z:.2f} m). Switching to SERVO state."
                    )
                    self.state = "SERVO"

        # OpenCV windows need a display; off by default (headless / SSH / CI).
        if os.environ.get('TERRAIN_DEBUG_CV', '') == '1':
            cv2.imshow("RGB Detection", overlay)
            cv2.imshow("Mask", object_mask.astype(np.uint8) * 255)
            cv2.waitKey(1)
    # ---------------------------------------------
    # Aruco detection and transformation to drone coordinates
    # ---------------------------------------------
    def aruco_cb(self, msg):
        import re
        match = re.match(r"Marker (\d+) detected at x:([-\d.]+)m, y:([-\d.]+)m, z:([-\d.]+)m", msg.data)
        if match:
            marker_id = int(match.group(1))
            x = float(match.group(2))
            y = float(match.group(3))
            z = float(match.group(4))
            # Transform to drone frame: x,y,z => y,x,z
            drone_x = y
            drone_y = x
            drone_z = z
            self.markers[marker_id] = (drone_x, drone_y, drone_z)
            self.get_logger().info(f"Updated Marker {marker_id}: x={drone_x}, y={drone_y}, z={drone_z}")

    # ---------------------------------------------
    # Timer Callback: Main State Machine
    # ---------------------------------------------
    def timer_callback(self):
        # Publish offboard control mode each cycle
        self.publish_offboard_control_mode()

        now = time.time()
        if self.state == 'WAIT_INTRINSICS' and now - self._last_wait_status_log >= 4.0:
            self._last_wait_status_log = now
            if self.fx is None:
                self.get_logger().info(
                    'State WAIT_INTRINSICS: waiting for /drone/front_depth/camera_info '
                    '(bridge starts ~10s after sim launch).'
                )
            else:
                self.get_logger().info(
                    'State WAIT_INTRINSICS: gates '
                    f'intrinsics=OK vlp={self._vlp_received} '
                    f'setpoint_cycles={self.offboard_setpoint_counter}/10 '
                    '(need vlp + 10 cycles before ARM). '
                    'If vlp stays false: check ROS_DOMAIN_ID, MicroXRCE agent, and '
                    '`ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort`.'
                )

        # State machine (do not elif-chain WAIT/ARM to the arm command: when counter>=10 in WAIT, the arm
        # `if` would be true and would skip the WAIT->ARM transition entirely.)

        if self.state == "WAIT_INTRINSICS":
            # Stream valid NED setpoints before Offboard (PX4 rejects z=0 at ground as invalid hold).
            if self._vlp_received:
                self.publish_trajectory_setpoint(
                    self.position[0], self.position[1], self.position[2], self._vlp_heading
                )
            else:
                self.publish_trajectory_setpoint(0.0, 0.0, -0.5, 0.0)
            # Battery status isn't always published/valid in SITL; don't block the whole mission on it.
            ready = (
                (self.fx is not None)
                and (self.fy is not None)
                and self._vlp_received
                and (self.offboard_setpoint_counter >= 10)
            )
            if ready:
                if self.battery_at_mission_start is None and self.battery_percent is not None:
                    self.battery_at_mission_start = self.battery_percent
                    self.get_logger().info(
                        f"Locked battery_at_mission_start: {self.battery_at_mission_start:.4f}"
                    )
                elif self.battery_percent is None and (self.offboard_setpoint_counter % 20 == 0):
                    self.get_logger().warn(
                        "Battery estimate not available (SITL). Continuing mission without energy logging."
                    )

                self.get_logger().info(
                    'Intrinsics + PX4 pose + 10 setpoint cycles OK. Moving to ARM_TAKEOFF.'
                )
                self.state = "ARM_TAKEOFF"
                self.start_time = time.time()

        elif self.state == "ARM_TAKEOFF":
            if self.takeoff_stage == 0:
                # Stage 1: Vertical takeoff to (0, 0, -5)
                target = [0.0, 0.0, -5.0]
                self.publish_trajectory_setpoint(target[0], target[1], target[2], self._vlp_heading)

                dx = self.position[0] - target[0]
                dy = self.position[1] - target[1]
                dz = self.position[2] - target[2]
                dist = math.sqrt(dx**2 + dy**2 + dz**2)

                if dist < 0.5:
                    self.get_logger().info("Vertical takeoff complete. Proceeding to circle entry point.")
                    self.takeoff_stage = 1

            elif self.takeoff_stage == 1:
                # Stage 2: Move to (15, 0, -5)
                target = [15.0, 0.0, -5.0]
                self.publish_trajectory_setpoint(target[0], target[1], target[2], self._vlp_heading)

                dx = self.position[0] - target[0]
                dy = self.position[1] - target[1]
                dz = self.position[2] - target[2]
                dist = math.sqrt(dx**2 + dy**2 + dz**2)

                if dist < 0.5:
                    # Set theta based on actual position
                    self.theta = math.atan2(self.position[1], self.position[0])
                    self.get_logger().info("Reached circle entry point. Switching to CIRCLE.")
                    self.state = "CIRCLE"

        elif self.state == "CIRCLE":
            # Circle flight: yaw tangent to motion (circle_speed sign picks CW vs CCW).
            x = self.circle_radius * math.cos(self.theta)
            y = self.circle_radius * math.sin(self.theta)
            z = self.altitude
            tang = math.pi / 2.0 if self.circle_speed >= 0.0 else -math.pi / 2.0
            yaw = self.theta + tang
            self.publish_trajectory_setpoint(x, y, z, yaw)
            self.theta += self.circle_speed

        elif self.state == "SERVO":
            # Start the timer only once
            if self.servo_start_time is None:
                self.servo_start_time = time.time()

            # Check if we have recent depth data
            current_distance = None
            if len(self.points_buffer) > 0:
                _, _, Z = self.points_buffer[-1]
                current_distance = Z

            if current_distance is None:
                # Timeout logic: give up after 5 seconds
                if time.time() - self.servo_start_time > 12.0:
                    self.get_logger().warn("Object not found within timeout. Returning to CIRCLE.")
                    
                    # Clear stale detection data
                    self.points_buffer.clear()
                    
                    # Reset timer and return to CIRCLE
                    self.servo_start_time = None
                    drone_x, drone_y, _ = self.position
                    self.theta = math.atan2(drone_y, drone_x)
                    self.state = "CIRCLE"

                else:
                    # Keep hovering during search
                    self.publish_trajectory_setpoint(
                        self.position[0], self.position[1], self.altitude, self._vlp_heading
                    )
            else:
                # Move along local x toward range goal; clamp correction so bad depth cannot
                # command a sprint to the horizon.
                Z_use = float(np.clip(current_distance, 2.0, 35.0))
                distance_error = self.desired_distance - Z_use
                drone_x, drone_y, _ = self.position
                gain = 0.5
                raw_dx = distance_error * gain
                max_step = 1.2
                dx = max(-max_step, min(max_step, raw_dx))

                target_x = drone_x - dx
                target_y = drone_y
                target_z = self.altitude
                # Stay near the mapped survey circle (origin-centered) — extra safety clamp.
                r_tgt = math.hypot(target_x, target_y)
                r_max = self.circle_radius + 8.0
                if r_tgt > r_max and r_tgt > 1e-6:
                    scale = r_max / r_tgt
                    target_x *= scale
                    target_y *= scale

                self.publish_trajectory_setpoint(
                    target_x, target_y, target_z, self._vlp_heading
                )

                # If within tolerance, hover to measure
                if abs(distance_error) < self.distance_tolerance:
                    self.get_logger().info("Reached ~15m from cylinder. Going to HOVER to measure.")
                    self.hover_start_time = time.time()
                    self.servo_start_time = None  # Reset for next time
                    self.state = "HOVER"

        elif self.state == "HOVER":
            # Maintain current position at the hover altitude
            self.publish_trajectory_setpoint(
                self.position[0], self.position[1], self.altitude, self._vlp_heading
            )

            # Check if 5 seconds have passed since entering HOVER
            if time.time() - self.hover_start_time >= 7.0:
                self.get_logger().info("7s hover done. Checking measurement.")

                # Check if we collected bounding-box data
                if len(self.points_buffer) > 0:
                    widths, heights, depths = zip(*self.points_buffer)
                    median_w = statistics.median(widths)
                    median_h = statistics.median(heights)
                    self.get_logger().info(
                        f"[Cylinder Dimensions] Width={median_w:.2f} m, Height={median_h:.2f} m"
                    )

                    # Clear buffer for next object
                    self.points_buffer.clear()

                    tol = self.cylinder_dim_tolerance
                    n_cat = len(self.measured_cylinders)

                    def _matches_any(w, h):
                        for w_old, h_old in self.measured_cylinders:
                            if abs(w_old - w) < tol and abs(h_old - h) < tol:
                                return True
                        return False

                    if n_cat >= 2:
                        # Land only after re-acquiring the tallest catalogued cylinder (by apparent height).
                        tw, th = max(self.measured_cylinders, key=lambda wh: wh[1])
                        match_tallest = abs(median_w - tw) < tol and abs(median_h - th) < tol
                        if match_tallest:
                            self.get_logger().info(
                                'Re-acquired tallest catalogued cylinder — ArUco hover and landing.'
                            )
                            self._aruco_hover_xy = (self.position[0], self.position[1])
                            self._aruco_hover_enter_time = time.time()
                            self.aruco_hover_start_time = None
                            self.state = "ARUCO_HOVER"
                        else:
                            if not _matches_any(median_w, median_h):
                                self.measured_cylinders.append((median_w, median_h))
                                self.get_logger().info(
                                    f'Additional cylinder sample: w={median_w:.2f} m h={median_h:.2f} m'
                                )
                            else:
                                self.get_logger().info(
                                    'Measurement matches a known cylinder (not tallest target); continuing.'
                                )
                            self.detection_cooldown_until = time.time() + 5.0
                            drone_x, drone_y, _ = self.position
                            self.theta = math.atan2(drone_y, drone_x)
                            self.state = "CIRCLE"

                    elif n_cat == 1:
                        w0, h0 = self.measured_cylinders[0]
                        if abs(w0 - median_w) < tol and abs(h0 - median_h) < tol:
                            self.get_logger().info(
                                'Still observing first cylinder; continuing circle for a second structure.'
                            )
                            self.detection_cooldown_until = time.time() + 5.0
                            drone_x, drone_y, _ = self.position
                            self.theta = math.atan2(drone_y, drone_x)
                            self.state = "CIRCLE"
                        else:
                            self.measured_cylinders.append((median_w, median_h))
                            tw, th = max(self.measured_cylinders, key=lambda wh: wh[1])
                            self.get_logger().info(
                                f'Second cylinder catalogued (w={median_w:.2f}, h={median_h:.2f}). '
                                f'Current tallest h≈{th:.2f} m — circle until tallest is re-acquired.'
                            )
                            self.detection_cooldown_until = time.time() + 6.0
                            drone_x, drone_y, _ = self.position
                            self.theta = math.atan2(drone_y, drone_x)
                            self.state = "CIRCLE"

                    else:
                        # First structure — record and keep mapping/searching.
                        self.measured_cylinders.append((median_w, median_h))
                        self.get_logger().info(
                            f'First cylinder catalogued (w={median_w:.2f}, h={median_h:.2f}). '
                            'Continuing search for second cylinder.'
                        )
                        self.detection_cooldown_until = time.time() + 6.0
                        drone_x, drone_y, _ = self.position
                        self.theta = math.atan2(drone_y, drone_x)
                        self.state = "CIRCLE"
                else:
                    self.get_logger().warn("No data in points_buffer. Resuming circle anyway.")
                    # Return to circle state
                    self.state = "CIRCLE"

        elif self.state == "ARUCO_HOVER":
            # IMPORTANT: keep current XY — (0,0,-20) flies the drone to map origin and breaks landing.
            hx = self._aruco_hover_xy[0] if self._aruco_hover_xy else self.position[0]
            hy = self._aruco_hover_xy[1] if self._aruco_hover_xy else self.position[1]
            target_z = -20.0
            self.publish_trajectory_setpoint(hx, hy, target_z, self._vlp_heading)

            z_enter = self._aruco_hover_enter_time or now
            alt_ok = abs(self.position[2] - target_z) < 2.0
            stale = (now - z_enter) > 40.0
            if self.aruco_hover_start_time is None:
                if alt_ok or stale:
                    self.aruco_hover_start_time = now
                    self.get_logger().info(
                        f'ArUco altitude band: target_z={target_z:.1f} actual z={self.position[2]:.2f} '
                        f'(alt_ok={alt_ok}, stale_timeout={stale}). Holding {self._aruco_hover_hold_sec:.0f}s...'
                    )
            elif now - self.aruco_hover_start_time >= self._aruco_hover_hold_sec:
                self.get_logger().info('ArUco hover settle complete. Selecting marker...')
                self.state = "ARUCO_SELECT"
                self._aruco_select_enter_time = now
                self.land_marker_id = None

        elif self.state == "ARUCO_SELECT":
            # Prefer two markers for “tallest” (min z in camera frame); one marker is enough to proceed.
            # Prefer marker ID 0 (tall cylinder in course assets) when both markers were seen.
            if len(self.markers) >= 1:
                if 0 in self.markers:
                    best_marker_id = 0
                    _mx, _my, mz = self.markers[0]
                    min_z = mz
                else:
                    best_marker_id = None
                    min_z = float('inf')
                    for mid, (_mx, _my, mz) in self.markers.items():
                        if mz < min_z:
                            min_z = mz
                            best_marker_id = mid
                self.land_marker_id = best_marker_id
                self.get_logger().info(
                    f'Selected Marker {best_marker_id} for landing '
                    f'(min marker z={min_z:.3f} m among {len(self.markers)} seen).'
                )
                self._aruco_move_start_time = now
                self.state = "ARUCO_MOVE"
            elif self._aruco_select_enter_time is not None and (
                now - self._aruco_select_enter_time
            ) > 45.0:
                self.get_logger().warn(
                    'No ArUco poses after timeout — NAV_LAND at current horizontal position.'
                )
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self._land_command_time = now
                self.state = "ARUCO_LAND"

        elif self.state == "ARUCO_MOVE":
            # Incremental pursuit using latest marker offsets (camera→drone remapping from aruco_cb).
            mid = self.land_marker_id
            if mid is None or mid not in self.markers:
                self.publish_trajectory_setpoint(
                    self.position[0], self.position[1], -20.0, self._vlp_heading
                )
            else:
                mx, my, _mz = self.markers[mid]
                gain = 0.7
                tx = self.position[0] + mx * gain
                ty = self.position[1] + my * gain
                tz = -20.0
                self.publish_trajectory_setpoint(tx, ty, tz, self._vlp_heading)

                horiz_err = math.hypot(mx, my)
                move_elapsed = (
                    now - self._aruco_move_start_time
                    if self._aruco_move_start_time is not None
                    else 0.0
                )
                if horiz_err < 0.22 or move_elapsed > 75.0:
                    self.get_logger().info(
                        f'Approach end: horiz_err={horiz_err:.3f} m, elapsed={move_elapsed:.1f}s → NAV_LAND'
                    )
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    self._land_command_time = now
                    self.state = "ARUCO_LAND"

        elif self.state == "ARUCO_LAND":
            # Wait for touchdown in SITL before disarm (immediate disarm leaves the vehicle airborne).
            if self._land_command_time is None:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self._land_command_time = now
                self.get_logger().info('NAV_LAND issued; waiting before disarm...')
            elif now - self._land_command_time >= 20.0:
                if not self._aruco_land_disarm_done:
                    self.get_logger().info('Touchdown window elapsed; disarming.')
                    self.publish_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
                    )
                    self._aruco_land_disarm_done = True
                self.state = "COMPLETE"

        elif self.state == "COMPLETE":
            self.get_logger().info("Mission complete.")
            sys.stdout.flush()
            sys.stderr.flush()

            if self.battery_at_mission_end is None and self.battery_percent is not None:
                self.battery_at_mission_end = self.battery_percent
                self.get_logger().info(f"Captured battery_at_mission_end: {self.battery_at_mission_end:.4f}")

            if self.start_time is not None:
                mission_duration = time.time() - self.start_time
                self.get_logger().info(f"Mission Duration: {mission_duration:.2f} seconds")

                if self.battery_at_mission_start is not None and self.battery_at_mission_end is not None:
                    used = (self.battery_at_mission_start - self.battery_at_mission_end) * 100.0
                    self.get_logger().info(f"Battery Used: {used:.3f}%")
                else:
                    self.get_logger().warn("Missing start/end battery data!")

            self.state = "DONE"

        elif self.state == "DONE":
            sys.stdout.flush()
            sys.stderr.flush()
            rclpy.shutdown()
            pass

        # Run after state updates so the same tick can WAIT->ARM then arm here.
        if self.offboard_setpoint_counter >= 10 and not self._initial_arm_done:
            if self.state != "WAIT_INTRINSICS":
                for _ in range(3):
                    self.engage_offboard_mode()
                for _ in range(3):
                    self.arm()
                self._initial_arm_done = True

        if self.state in self._TELEM_STATES:
            do_log = (
                self._last_pose_telemetry_log is None
                or (now - self._last_pose_telemetry_log) >= self._pose_telemetry_interval_sec
            )
            if do_log:
                self._last_pose_telemetry_log = now
                px, py, pz = self.position[0], self.position[1], self.position[2]
                extra = ''
                if self.state == 'ARM_TAKEOFF':
                    if self.takeoff_stage == 0:
                        tx, ty, tz = 0.0, 0.0, -5.0
                    else:
                        tx, ty, tz = 15.0, 0.0, -5.0
                    dist = math.sqrt((px - tx) ** 2 + (py - ty) ** 2 + (pz - tz) ** 2)
                    extra = f' takeoff_stage={self.takeoff_stage} dist_to_subgoal={dist:.2f}m'
                self.get_logger().info(
                    f'TELEM state={self.state} pos=({px:.2f},{py:.2f},{pz:.2f}){extra}'
                )

        self.offboard_setpoint_counter += 1

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_cmd_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0
        )
        self.get_logger().info("Offboard mode command sent")


def main(args=None):
    rclpy.init(args=args)
    node = CylinderMission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Interrupted, shutting down.")
    finally:
        # `rclpy.shutdown()` may already have been called (e.g. by state machine / launch shutdown).
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
