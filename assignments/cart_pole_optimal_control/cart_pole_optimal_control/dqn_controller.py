#!/usr/bin/env python3

import os
from collections import deque

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import numpy as np
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from cart_pole_optimal_control.dqn_model import QNetwork, ACTIONS, normalize_state

def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class DQNController(Node):
    def __init__(self):
        super().__init__('dqn_controller')

        self.declare_parameter('model_path', os.path.expanduser('~/dqn_cartpole_eq.pt'))
        model_path = self.get_parameter('model_path').value

        self.model = QNetwork()
        self.model.load_state_dict(torch.load(model_path, map_location='cpu'))
        self.model.eval()

        self.state = None
        self.state_initialized = False
        self.raw_state = None
        self.theta_offset = None
        self.start_time = None
        self.simulation_done = False

        self._last_uninitialized_warn_time = 0.0
        self._last_eq_warn_time = 0.0

        self.MAX_SIMULATION_TIME = 120.0
        self.TERMINATION_GRACE_SEC = 1.0

        self.last_control = 0.0
        self.control_count = 0

        self.earthquake_forces = deque()
        self.time_steps = deque()
        self.cart_positions = deque()
        self.pole_angles = deque()
        self.control_forces = deque()

        self.force_pub = self.create_publisher(
            Float64,
            '/model/cart_pole/joint/cart_to_base/cmd_force',
            10
        )

        joint_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            joint_state_qos
        )

        self.earthquake_sub = self.create_subscription(
            Float64,
            '/earthquake_force',
            self.earthquake_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz
        self.get_logger().info(f'DQN controller loaded model from: {model_path}')
        self.get_logger().info(f'DQN action set: {ACTIONS.tolist()}')

    def joint_state_callback(self, msg):
        try:
            cart_idx = msg.name.index('cart_to_base')
            pole_idx = msg.name.index('pole_joint')

            raw_x = float(msg.position[cart_idx])
            raw_xdot = float(msg.velocity[cart_idx])
            raw_theta = float(msg.position[pole_idx])
            raw_thetadot = float(msg.velocity[pole_idx])

            # On first valid state, define current pole angle as the local upright reference
            if self.theta_offset is None:
                self.theta_offset = raw_theta

            theta_corrected = wrap_angle(raw_theta - self.theta_offset)

            self.raw_state = np.array([
                raw_x,
                raw_xdot,
                raw_theta,
                raw_thetadot
            ], dtype=np.float32)

            self.state = np.array([
                raw_x,
                raw_xdot,
                theta_corrected,
                raw_thetadot
            ], dtype=np.float32)

            if not self.state_initialized:
                self.state_initialized = True
                self.start_time = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info(
                    f'Initial raw state: cart_pos={raw_x:.3f}, '
                    f'cart_vel={raw_xdot:.3f}, '
                    f'pole_angle_raw={raw_theta:.3f} rad ({np.degrees(raw_theta):.2f} deg), '
                    f'pole_vel={raw_thetadot:.3f}'
                )
                self.get_logger().info(
                    f'Using theta_offset={self.theta_offset:.3f} rad '
                    f'({np.degrees(self.theta_offset):.2f} deg)'
                )

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to parse joint states: {e}')

    def earthquake_callback(self, msg):
        if self.state_initialized:
            self.earthquake_forces.append(msg.data)
        else:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_eq_warn_time >= 1.0:
                self.get_logger().warn("Received earthquake force before state was initialized.")
                self._last_eq_warn_time = now

    def print_metrics(self):
        duration = self.time_steps[-1] if self.time_steps else 0.0
        max_cart_displacement = max(map(abs, self.cart_positions), default=0.0)
        max_pole_deviation = max(map(abs, self.pole_angles), default=0.0)
        avg_control_effort = np.mean(np.abs(self.control_forces)) if self.control_forces else 0.0
        stability_score = max(
            0,
            10 - (max_cart_displacement * 2) - (max_pole_deviation / 5) - (avg_control_effort / 20)
        )

        self.get_logger().info(f"DQN action set: {ACTIONS.tolist()}")
        self.get_logger().info(f"Duration of stable operation: {duration:.2f} s")
        self.get_logger().info(f"Maximum cart displacement: {max_cart_displacement:.3f} m")
        self.get_logger().info(f"Maximum pendulum angle deviation: {max_pole_deviation:.3f}°")
        self.get_logger().info(f"Average control effort: {avg_control_effort:.3f} N")
        self.get_logger().info(f"Stability score: {stability_score:.2f}/10")

    def plot_results(self):
        out = os.path.join(os.path.expanduser("~"), "cart_pole_dqn_results.png")

        plt.figure(figsize=(12, 10))

        plt.subplot(2, 2, 1)
        plt.plot(self.time_steps, self.cart_positions, label='Cart Position (m)', color='b')
        plt.xlabel('Time (s)')
        plt.ylabel('Cart Position (m)')
        plt.legend()

        plt.subplot(2, 2, 2)
        plt.plot(self.time_steps, self.pole_angles, label='Pole Angle (°)', color='r')
        plt.xlabel('Time (s)')
        plt.ylabel('Pole Angle (°)')
        plt.legend()

        plt.subplot(2, 2, 3)
        plt.plot(self.time_steps, self.earthquake_forces, label='Earthquake Force (N)', color='g')
        plt.xlabel('Time (s)')
        plt.ylabel('Earthquake Force (N)')
        plt.legend()

        plt.subplot(2, 2, 4)
        plt.plot(self.time_steps, self.control_forces, label='Control Force (N)', color='m')
        plt.xlabel('Time (s)')
        plt.ylabel('Control Force (N)')
        plt.legend()

        plt.tight_layout()
        plt.savefig(out, dpi=150)
        plt.close()
        return out

    def control_loop(self):
        if self.simulation_done:
            return

        if not self.state_initialized:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_uninitialized_warn_time >= 1.0:
                self.get_logger().warn('State not initialized yet')
                self._last_uninitialized_warn_time = now
            return

        s = normalize_state(self.state)
        s_tensor = torch.tensor(s, dtype=torch.float32).unsqueeze(0)

        with torch.no_grad():
            q_values = self.model(s_tensor)
            action_idx = int(torch.argmax(q_values, dim=1).item())

        force = float(ACTIONS[action_idx])

        msg = Float64()
        msg.data = force
        self.force_pub.publish(msg)

        self.last_control = force
        self.control_count += 1

        current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        self.time_steps.append(current_time)
        self.cart_positions.append(float(self.state[0]))
        self.pole_angles.append(float(np.degrees(self.state[2])))
        self.control_forces.append(force)

        if len(self.earthquake_forces) < len(self.time_steps):
            self.earthquake_forces.append(self.earthquake_forces[-1] if self.earthquake_forces else 0.0)

        if current_time < self.TERMINATION_GRACE_SEC:
            return

        if (
            abs(self.state[0]) > 2.5
            or abs(self.state[2]) > np.radians(45)
            or current_time >= self.MAX_SIMULATION_TIME
        ):
            self.simulation_done = True
            self.timer.cancel()
            self.get_logger().warn(
                f"Simulation ended: cart_x={self.state[0]:.2f}m, "
                f"pole_angle={np.degrees(self.state[2]):.2f}°, "
                f"duration={current_time:.2f}s"
            )
            self.print_metrics()
            try:
                path = self.plot_results()
                if path:
                    self.get_logger().info(f"Saved plot to {path}")
            except Exception as plot_error:
                self.get_logger().warn(f"Plot generation failed: {plot_error}")
            if rclpy.ok():
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DQNController()
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