#!/usr/bin/env python3

import os
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from cart_pole_optimal_control.dqn_model import QNetwork, ACTIONS, normalize_state

class DQNController(Node):
    def __init__(self):
        super().__init__('dqn_controller')

        self.declare_parameter('model_path', os.path.expanduser('~/dqn_cartpole.pt'))
        model_path = self.get_parameter('model_path').value

        self.model = QNetwork()
        self.model.load_state_dict(torch.load(model_path, map_location='cpu'))
        self.model.eval()

        self.state = None

        self.force_pub = self.create_publisher(
            Float64,
            '/model/cart_pole/joint/cart_to_base/cmd_force',
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz
        self.get_logger().info(f'DQN controller loaded model from: {model_path}')

    def joint_state_callback(self, msg):
        try:
            cart_idx = msg.name.index('cart_to_base')
            pole_idx = msg.name.index('pole_joint')

            self.state = np.array([
                msg.position[cart_idx],
                msg.velocity[cart_idx],
                msg.position[pole_idx],
                msg.velocity[pole_idx]
            ], dtype=np.float32)

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to parse joint states: {e}')

    def control_loop(self):
        if self.state is None:
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