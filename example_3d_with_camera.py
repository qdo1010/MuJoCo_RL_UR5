#!/usr/bin/env python3
"""
Example to show both 3D viewer and camera observations.
This script must be run with mjpython on macOS.
"""

import gymnasium as gym
import numpy as np
import time
from termcolor import colored

def main():
    print("=" * 60)
    print("UR5 Gripper 3D Simulation + Camera Observations")
    print("=" * 60)
    print("Note: This script must be run with 'mjpython' on macOS")
    print("You should see:")
    print("1. A 3D MuJoCo viewer window showing the UR5 robot")
    print("2. Camera observation windows showing RGB and depth")
    print("=" * 60)
    
    # Create environment with both rendering and camera observations
    env = gym.make("gym_grasper:Grasper-v0", show_obs=True, render=True)
    
    print("Environment created successfully!")
    
    # Get the unwrapped environment
    unwrapped_env = env.unwrapped
    
    try:
        # Reset the environment
        obs, info = env.reset()
        print("Environment reset. Robot is in initial position.")
        
        # Show observation information
        print(f"\nObservation keys: {list(obs.keys())}")
        print(f"RGB shape: {obs['rgb'].shape}")
        print(f"Depth shape: {obs['depth'].shape}")
        
        # Run a few episodes with both 3D viewer and camera observations
        for episode in range(3):
            print(f"\n{'='*50}")
            print(f"EPISODE {episode + 1}")
            print(f"{'='*50}")
            
            obs, info = env.reset()
            
            for step in range(3):
                print(f"\nStep {step + 1}:")
                
                # Sample a random action
                action = env.action_space.sample()
                print(f"Action: {action}")
                
                # Execute the action
                obs, reward, terminated, truncated, info = env.unwrapped.step(
                    action, 
                    record_grasps=False, 
                    markers=False
                )
                
                print(f"Reward: {reward}")
                
                # Wait to see the action
                time.sleep(2)
        
        print("\nSimulation complete!")
        print("Both the 3D viewer and camera windows should still be open.")
        print("Press Ctrl+C to exit...")
        
        # Keep everything open
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nExiting simulation...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        env.close()
        print("Environment closed.")

if __name__ == "__main__":
    main()
