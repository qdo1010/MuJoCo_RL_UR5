#!/usr/bin/env python3
"""
Example to show 3D viewer AND save camera observations to files.
This avoids OpenCV window conflicts while still showing both views.
Run with: mjpython example_3d_with_saved_camera.py
"""

import gymnasium as gym
import numpy as np
import time
import cv2
import os
from termcolor import colored

def save_observation_images(obs, episode, step):
    """Save observation images to files instead of displaying them"""
    # Create directory for saved images
    os.makedirs("camera_observations", exist_ok=True)
    
    # Save RGB image
    rgb_filename = f"camera_observations/episode_{episode}_step_{step}_rgb.png"
    cv2.imwrite(rgb_filename, obs['rgb'])
    
    # Save depth image (normalize for visualization)
    depth_normalized = (obs['depth'] / obs['depth'].max() * 255).astype(np.uint8)
    depth_filename = f"camera_observations/episode_{episode}_step_{step}_depth.png"
    cv2.imwrite(depth_filename, depth_normalized)
    
    print(f"Saved camera images: {rgb_filename}, {depth_filename}")

def main():
    print("=" * 60)
    print("UR5 Gripper 3D Simulation + Saved Camera Observations")
    print("=" * 60)
    print("Note: This script must be run with 'mjpython' on macOS")
    print("You should see:")
    print("1. A 3D MuJoCo viewer window showing the UR5 robot")
    print("2. Camera observations saved to 'camera_observations/' folder")
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
        
        # Save initial observation
        save_observation_images(obs, 0, 0)
        
        # Run a few episodes with both 3D viewer and saved camera observations
        for episode in range(3):
            print(f"\n{'='*50}")
            print(f"EPISODE {episode + 1}")
            print(f"{'='*50}")
            
            obs, info = env.reset()
            save_observation_images(obs, episode + 1, 0)
            
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
                
                # Save the observation after the action
                save_observation_images(obs, episode + 1, step + 1)
                
                # Wait to see the action in 3D viewer
                time.sleep(2)
        
        print("\nSimulation complete!")
        print("The 3D viewer should still be open.")
        print("Camera observations have been saved to 'camera_observations/' folder.")
        print("You can open these images to see what the robot's cameras see.")
        print("Press Ctrl+C to exit...")
        
        # Keep the viewer open
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
