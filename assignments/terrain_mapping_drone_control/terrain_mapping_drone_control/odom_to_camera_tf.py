#!/usr/bin/env python3
"""Publish TF odom -> oakd_rgb and Odometry for RTAB-Map from Gazebo / PX4 bridge odometry."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import tf2_ros

# Camera origin relative to multicopter base_link (x500_depth_mono + OakD joint).
CAM_IN_BASE = np.array([0.12, 0.03, 0.242], dtype=np.float64)

ODOM_FRAME = 'odom'
# Must match the frame_id used by the Gazebo camera topics.
CAM_FRAME = 'OakD-Lite-Modify/base_link'


def _rotate_vec_by_unit_quat(v: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Rotate vector v by unit quaternion q = [w, x, y, z]."""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    qv = np.array([x, y, z], dtype=np.float64)
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


class OdomToCameraTf(Node):
    def __init__(self):
        super().__init__('odom_to_camera_tf')
        # Launch files typically inject `use_sim_time`; don't fail if it is already declared.
        try:
            self.declare_parameter('use_sim_time', True)
        except ParameterAlreadyDeclaredException:
            pass
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Odometry, '/drone/gz_odometry', self._odom_cb, qos)
        self._pub = self.create_publisher(Odometry, '/drone/rtab_odom', qos)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        q = np.array([o.w, o.x, o.y, o.z], dtype=np.float64)
        nq = float(np.linalg.norm(q))
        if nq < 1e-6:
            return
        q /= nq

        base = np.array([p.x, p.y, p.z], dtype=np.float64)
        cam = base + _rotate_vec_by_unit_quat(CAM_IN_BASE, q)

        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = ODOM_FRAME
        t.child_frame_id = CAM_FRAME
        t.transform.translation.x = float(cam[0])
        t.transform.translation.y = float(cam[1])
        t.transform.translation.z = float(cam[2])
        t.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)

        out = Odometry()
        out.header.stamp = now
        out.header.frame_id = ODOM_FRAME
        out.child_frame_id = CAM_FRAME
        out.pose.pose.position.x = float(cam[0])
        out.pose.pose.position.y = float(cam[1])
        out.pose.pose.position.z = float(cam[2])
        out.pose.pose.orientation = msg.pose.pose.orientation
        out.twist = msg.twist
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToCameraTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
