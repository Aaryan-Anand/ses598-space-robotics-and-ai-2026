#!/usr/bin/env python3

import os
import math
import random
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from cart_pole_optimal_control.dqn_model import QNetwork, ReplayBuffer, ACTIONS, normalize_state

GAMMA = 0.99
LR = 1e-3
BATCH_SIZE = 256
BUFFER_SIZE = 50000
MIN_REPLAY_SIZE = 2000
TARGET_UPDATE_EVERY = 500
NUM_EPISODES = 400
MAX_STEPS = 1000

EPS_START = 1.0
EPS_END = 0.05
EPS_DECAY = 0.995

class SimpleCartPoleEnv:
    def __init__(self):
        self.dt = 0.02
        self.g = 9.81
        self.M = 1.0
        self.m = 1.0
        self.L = 1.0

        self.x_limit = 2.5
        self.theta_limit = math.radians(45)

        self.state = None

    def reset(self):
        self.state = np.array([
            np.random.uniform(-0.05, 0.05),   # x
            np.random.uniform(-0.05, 0.05),   # xdot
            np.random.uniform(-0.05, 0.05),   # theta
            np.random.uniform(-0.05, 0.05)    # thetadot
        ], dtype=np.float32)
        return self.state.copy()

    def step(self, u):
        x, xdot, theta, thetadot = self.state

        # crude nonlinear-ish dynamics
        sin_t = np.sin(theta)
        cos_t = np.cos(theta)
        total_mass = self.M + self.m

        temp = (u + self.m * self.L * thetadot**2 * sin_t) / total_mass
        theta_acc = (self.g * sin_t - cos_t * temp) / (
            self.L * (4.0/3.0 - self.m * cos_t**2 / total_mass)
        )
        x_acc = temp - self.m * self.L * theta_acc * cos_t / total_mass

        x = x + self.dt * xdot
        xdot = xdot + self.dt * x_acc
        theta = theta + self.dt * thetadot
        thetadot = thetadot + self.dt * theta_acc

        self.state = np.array([x, xdot, theta, thetadot], dtype=np.float32)

        reward = (
            1.0
            - 2.0 * (theta / 0.35) ** 2
            - 0.5 * (x / 2.5) ** 2
            - 0.05 * (xdot / 3.0) ** 2
            - 0.05 * (thetadot / 2.0) ** 2
            - 0.001 * (u ** 2)
        )

        done = abs(x) > self.x_limit or abs(theta) > self.theta_limit

        if done:
            reward -= 50.0

        return self.state.copy(), reward, done, {}

def train():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    print(f"Using device: {device}")

    print(f"Using device: {device}")

    env = SimpleCartPoleEnv()
    online_net = QNetwork().to(device)
    target_net = QNetwork().to(device)
    target_net.load_state_dict(online_net.state_dict())

    optimizer = optim.Adam(online_net.parameters(), lr=LR)
    replay = ReplayBuffer(BUFFER_SIZE)

    epsilon = EPS_START
    rewards_history = []
    losses = []
    step_count = 0
    best_reward = -1e9

    for episode in range(NUM_EPISODES):
        state = normalize_state(env.reset())
        episode_reward = 0.0

        for _ in range(MAX_STEPS):
            step_count += 1

            if random.random() < epsilon:
                action_idx = random.randrange(len(ACTIONS))
            else:
                with torch.no_grad():
                    s = torch.tensor(state, dtype=torch.float32, device=device).unsqueeze(0)
                    action_idx = int(torch.argmax(online_net(s), dim=1).item())

            u = float(ACTIONS[action_idx])
            next_raw_state, reward, done, _ = env.step(u)
            next_state = normalize_state(next_raw_state)

            replay.push(state, action_idx, reward, next_state, done)

            state = next_state
            episode_reward += reward

            if len(replay) >= MIN_REPLAY_SIZE and step_count % 4 == 0:
                states, actions, rewards, next_states, dones = replay.sample(BATCH_SIZE)

                states = torch.tensor(states, dtype=torch.float32, device=device)
                actions = torch.tensor(actions, dtype=torch.int64, device=device).unsqueeze(1)
                rewards = torch.tensor(rewards, dtype=torch.float32, device=device).unsqueeze(1)
                next_states = torch.tensor(next_states, dtype=torch.float32, device=device)
                dones = torch.tensor(dones, dtype=torch.float32, device=device).unsqueeze(1)

                q_values = online_net(states).gather(1, actions)

                with torch.no_grad():
                    max_next_q = target_net(next_states).max(dim=1, keepdim=True)[0]
                    target_q = rewards + GAMMA * max_next_q * (1.0 - dones)

                loss = nn.MSELoss()(q_values, target_q)

                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

                losses.append(loss.item())

                if step_count % TARGET_UPDATE_EVERY == 0:
                    target_net.load_state_dict(online_net.state_dict())

            if done:
                break

        rewards_history.append(episode_reward)

        epsilon = max(EPS_END, epsilon * EPS_DECAY)

        if episode_reward > best_reward:
            best_reward = episode_reward
            torch.save(online_net.state_dict(), os.path.expanduser("~/dqn_cartpole.pt"))

        print(f"Episode {episode+1}/{NUM_EPISODES} | reward={episode_reward:.2f} | epsilon={epsilon:.3f}")

    plt.figure(figsize=(10, 4))
    plt.plot(rewards_history)
    plt.xlabel("Episode")
    plt.ylabel("Reward")
    plt.title("DQN Training Reward")
    plt.tight_layout()
    plt.savefig(os.path.expanduser("~/dqn_training_rewards.png"), dpi=150)
    plt.close()

    if losses:
        plt.figure(figsize=(10, 4))
        plt.plot(losses)
        plt.xlabel("Training Step")
        plt.ylabel("Loss")
        plt.title("DQN Training Loss")
        plt.tight_layout()
        plt.savefig(os.path.expanduser("~/dqn_training_loss.png"), dpi=150)
        plt.close()

def main():
    train()

if __name__ == "__main__":
    main()