#!/usr/bin/env python3

import gymnasium as gym
import numpy as np
from termcolor import colored
import time

# Create environment with both camera observations AND 3D viewer
env = gym.make("gym_grasper:Grasper-v0", show_obs=True, render=True)

N_EPISODES = 5  # Reduced for demo
N_STEPS = 3     # Reduced for demo

# Access the unwrapped environment to call print_info
env.unwrapped.print_info()

print("\n" + "="*60)
print("You should now see:")
print("1. A 3D MuJoCo viewer window showing the UR5 robot")
print("2. Camera observation windows showing RGB and depth")
print("3. Console output with action details")
print("="*60 + "\n")

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
        
        print(f"Reward: {reward}, Done: {done}")
        print("Press Enter to continue to next step, or Ctrl+C to exit...")
        
        # Wait for user input to see each step clearly
        try:
            input()
        except KeyboardInterrupt:
            print("\nExiting...")
            env.close()
            exit()

env.close()

print("Finished.")
