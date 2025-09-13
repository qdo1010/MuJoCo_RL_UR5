#!/usr/bin/env python3
"""
Example to show 3D viewer AND camera observations using matplotlib.
This might avoid OpenCV conflicts while showing both views.
Run with: mjpython example_3d_with_matplotlib.py
"""

import gymnasium as gym
import numpy as np
import time
import matplotlib.pyplot as plt
from termcolor import colored

def display_observations(obs, episode, step):
    """Display observations using matplotlib instead of OpenCV"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    
    # Display RGB image
    ax1.imshow(obs['rgb'])
    ax1.set_title(f'RGB Camera - Episode {episode}, Step {step}')
    ax1.axis('off')
    
    # Display depth image
    ax2.imshow(obs['depth'], cmap='viridis')
    ax2.set_title(f'Depth Camera - Episode {episode}, Step {step}')
    ax2.axis('off')
    
    plt.tight_layout()
    plt.show(block=False)
    plt.pause(0.1)  # Brief pause to update the display

def main():
    print("=" * 60)
    print("UR5 Gripper 3D Simulation + Matplotlib Camera Display")
    print("=" * 60)
    print("Note: This script must be run with 'mjpython' on macOS")
    print("You should see:")
    print("1. A 3D MuJoCo viewer window showing the UR5 robot")
    print("2. Matplotlib windows showing camera observations")
    print("=" * 60)
    
    # Create environment with 3D viewer but NO OpenCV display
    env = gym.make("gym_grasper:Grasper-v0", show_obs=False, render=True)
    
    print("Environment created successfully!")
    print("You should see a 3D MuJoCo viewer window with the UR5 robot.")
    print()
    
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
        
        # Display initial observation
        display_observations(obs, 0, 0)
        
        # Run a few episodes with both 3D viewer and matplotlib camera display
        for episode in range(2):  # Reduced for demo
            print(f"\n{'='*50}")
            print(f"EPISODE {episode + 1}")
            print(f"{'='*50}")
            
            obs, info = env.reset()
            display_observations(obs, episode + 1, 0)
            
            for step in range(2):  # Reduced for demo
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
                
                # Display the observation after the action
                display_observations(obs, episode + 1, step + 1)
                
                # Wait to see the action in 3D viewer
                time.sleep(3)
        
        print("\nSimulation complete!")
        print("The 3D viewer and matplotlib windows should still be open.")
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
        plt.close('all')
        print("Environment closed.")

if __name__ == "__main__":
    main()
