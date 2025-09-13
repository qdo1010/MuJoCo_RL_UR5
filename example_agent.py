#!/usr/bin/env python3

import gymnasium as gym
import numpy as np
from termcolor import colored
import time

env = gym.make("gym_grasper:Grasper-v0", show_obs=True, render=True)

N_EPISODES = 100
N_STEPS = 100

# Access the unwrapped environment to call print_info
env.unwrapped.print_info()

for episode in range(1, N_EPISODES + 1):
    obs, info = env.reset()
    for step in range(N_STEPS):
        print("#################################################################")
        print(
            colored("EPISODE {} STEP {}".format(episode, step + 1), color="white", attrs=["bold"])
        )
        print("#################################################################")
        action = env.action_space.sample()
        # action = [100,100] # multidiscrete
        # action = 20000 #discrete
        observation, reward, terminated, truncated, info = env.unwrapped.step(action, record_grasps=True)
        done = terminated or truncated
        # observation, reward, done, _ = env.step(action, record_grasps=True, render=True)

env.close()

print("Finished.")
