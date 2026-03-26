#!/usr/bin/env python3

import os

# Non-interactive backend: avoids GTK/SVG errors in headless or broken display setups.
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
from scipy import linalg
from collections import deque

class CartPoleLQRController(Node):
    def __init__(self):
        super().__init__('cart_pole_lqr_controller')
        
        # System parameters
        self.M = 1.0  # Mass of cart (kg)
        self.m = 1.0  # Mass of pole (kg)
        self.L = 1.0  # Length of pole (m)
        self.g = 9.81  # Gravity (m/s^2)
        
        # State space matrices
        self.A = np.array([
            [0, 1, 0, 0],
            [0, 0, (self.m * self.g) / self.M, 0],
            [0, 0, 0, 1],
            [0, 0, ((self.M + self.m) * self.g) / (self.M * self.L), 0]
        ])
        
        self.B = np.array([
            [0],
            [1/self.M],
            [0],
            [-1/(self.M * self.L)]
        ])
        
        x_max=1.2
        xdot_max=3.0
        theta_max=np.deg2rad(6.0)
        thetadot_max=0.9
        u_max=15.0

        # LQR cost matrices
        self.Q = np.diag([
            (1/x_max**2),
            (1/xdot_max**2),
            (1/theta_max**2),
            (1/thetadot_max**2)
        ])  # State cost
        self.R = np.array([[1/u_max**2]])  # Control cost
        
        # Compute LQR gain matrix
        self.K = self.compute_lqr_gain()
        self.get_logger().info(f'LQR Gain Matrix: {self.K}')
        
        # Initialize state estimate
        self.x = np.zeros((4, 1))
        self.state_initialized = False
        self.last_control = 0.0
        self.control_count = 0
        self._last_uninitialized_warn_time = 0.0
        self._last_eq_warn_time = 0.0
        self.simulation_done = False
        self.earthquake_forces = deque()

        # Data storage for plotting
        self.time_steps = deque()
        self.cart_positions = deque()
        self.pole_angles = deque()
        self.control_forces = deque()
        self.start_time = None
        
        # Create publishers and subscribers
        self.cart_cmd_pub = self.create_publisher(Float64, '/model/cart_pole/joint/cart_to_base/cmd_force', 10)
        
        if self.cart_cmd_pub:
            self.get_logger().info('Force command publisher created successfully')
        
        # Match the republisher output topic and QoS to ensure the controller receives state.
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
        
        self.earthquake_sub = self.create_subscription(Float64, '/earthquake_force', self.earthquake_callback, 10)
        
        # Control loop timer
        self.timer = self.create_timer(0.01, self.control_loop)

        self.MAX_SIMULATION_TIME = 120.0  # Set to desired duration
        # Skip failure checks until physics and joint states settle (avoids instant stop at spawn).
        self.TERMINATION_GRACE_SEC = 1.0

        self.get_logger().info('Cart-Pole LQR Controller initialized')
    
    def compute_lqr_gain(self):
        """Compute the LQR gain matrix K."""
        P = linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ self.B.T @ P
        return K
    
    def joint_state_callback(self, msg):
        """Update state estimate from joint states."""
        try:
            cart_idx = msg.name.index('cart_to_base')
            pole_idx = msg.name.index('pole_joint')
            
            self.x = np.array([
                [msg.position[cart_idx]],
                [msg.velocity[cart_idx]],
                [msg.position[pole_idx]],
                [msg.velocity[pole_idx]]
            ])
            
            if not self.state_initialized:
                self.get_logger().info(f'Initial state: cart_pos={msg.position[cart_idx]:.3f}, cart_vel={msg.velocity[cart_idx]:.3f}, pole_angle={msg.position[pole_idx]:.3f}, pole_vel={msg.velocity[pole_idx]:.3f}')
                self.state_initialized = True
                self.start_time = self.get_clock().now().nanoseconds / 1e9
                
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to process joint states: {e}, msg={msg.name}')

    def earthquake_callback(self, msg):
        """Store earthquake force values."""
        if self.state_initialized:
            self.earthquake_forces.append(msg.data)
        else:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_eq_warn_time >= 1.0:
                self.get_logger().warn("Received earthquake force before state was initialized.")
                self._last_eq_warn_time = now

    def print_metrics(self):
        """Prints performance metrics after simulation ends."""
        duration = self.time_steps[-1] if self.time_steps else 0.0
        max_cart_displacement = max(map(abs, self.cart_positions), default=0.0)
        max_pole_deviation = max(map(abs, self.pole_angles), default=0.0)
        avg_control_effort = np.mean(np.abs(self.control_forces)) if self.control_forces else 0.0
        stability_score = max(0, 10 - (max_cart_displacement * 2) - (max_pole_deviation / 5) - (avg_control_effort / 20))


        self.get_logger().info(f"Q values: {self.Q.diagonal()}, R values: {self.R}")
        self.get_logger().info(f"Duration of stable operation: {duration:.2f} s")
        self.get_logger().info(f"Maximum cart displacement: {max_cart_displacement:.3f} m")
        self.get_logger().info(f"Maximum pendulum angle deviation: {max_pole_deviation:.3f}°")
        self.get_logger().info(f"Average control effort: {avg_control_effort:.3f} N")
        self.get_logger().info(f"Stability score: {stability_score:.2f}/10")



    def control_loop(self):
        """Compute and apply LQR control."""
        try:
            if self.simulation_done:
                return

            if not self.state_initialized:
                now = self.get_clock().now().nanoseconds / 1e9
                if now - self._last_uninitialized_warn_time >= 1.0:
                    self.get_logger().warn('State not initialized yet')
                    self._last_uninitialized_warn_time = now
                return

            u = -self.K @ self.x
            force = float(u[0])
            
            msg = Float64()
            msg.data = force
            self.cart_cmd_pub.publish(msg)
            
            self.last_control = force
            self.control_count += 1
            
            # Ensure time steps are synchronized
            current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
            self.time_steps.append(current_time)
            self.cart_positions.append(self.x[0, 0])
            self.pole_angles.append(np.degrees(self.x[2, 0]))
            self.control_forces.append(force)

            # Ensure earthquake force logging matches other data dimensions
            if len(self.earthquake_forces) < len(self.time_steps):
                self.earthquake_forces.append(self.earthquake_forces[-1] if self.earthquake_forces else 0.0)

            # **Termination Conditions** (after grace period so spawn pose does not instantly fail)
            if current_time < self.TERMINATION_GRACE_SEC:
                return

            if (
                abs(self.x[0, 0]) > 2.5
                or abs(self.x[2, 0]) > np.radians(45)
                or current_time >= self.MAX_SIMULATION_TIME
            ):
                self.simulation_done = True
                self.timer.cancel()
                self.get_logger().warn(f"Simulation ended: cart_x={self.x[0, 0]:.2f}m, pole_angle={np.degrees(self.x[2, 0]):.2f}°, duration={current_time:.2f}s")
                self.print_metrics()
                try:
                    path = self.plot_results()
                    if path:
                        self.get_logger().info(f"Saved plot to {path}")
                except Exception as plot_error:
                    self.get_logger().warn(f"Plot generation failed: {plot_error}")
                if rclpy.ok():
                    rclpy.shutdown()
                return

        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')

    def plot_results(self):
        """Generate plots for analysis; save PNG (no GUI required)."""
        out = os.path.join(os.path.expanduser("~"), "cart_pole_lqr_results.png")

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


def main(args=None):
    rclpy.init(args=args)
    controller = CartPoleLQRController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
