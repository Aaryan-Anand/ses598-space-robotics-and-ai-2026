import random
from collections import deque
import numpy as np
import torch
import torch.nn as nn

ACTIONS = np.array([-30, -20, -10, -5, 0, 5, 10, 20, 30], dtype=np.float32)

def normalize_state(state):
    x, xdot, theta, thetadot = state
    return np.array([
        x / 2.5,
        xdot / 5.0,
        theta / np.deg2rad(45.0),
        thetadot / 5.0
    ], dtype=np.float32)

class QNetwork(nn.Module):
    def __init__(self, state_dim=4, action_dim=len(ACTIONS)):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, action_dim)
        )

    def forward(self, x):
        return self.net(x)

class ReplayBuffer:
    def __init__(self, capacity=50000):
        self.buffer = deque(maxlen=capacity)

    def push(self, state, action_idx, reward, next_state, done):
        self.buffer.append((state, action_idx, reward, next_state, done))

    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        return (
            np.array(states, dtype=np.float32),
            np.array(actions, dtype=np.int64),
            np.array(rewards, dtype=np.float32),
            np.array(next_states, dtype=np.float32),
            np.array(dones, dtype=np.float32),
        )

    def __len__(self):
        return len(self.buffer)