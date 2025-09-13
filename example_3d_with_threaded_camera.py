#!/usr/bin/env python3
"""
Example to show 3D viewer AND camera observations using threading.
This tries to avoid OpenCV conflicts by running camera display in a separate thread.
Run with: mjpython example_3d_with_threaded_camera.py
"""

import gymnasium as gym
import numpy as np
import time
import cv2
import threading
from termcolor import colored
from queue import Queue

class CameraDisplayThread:
    def __init__(self):
        self.queue = Queue()
        self.running = True
        self.thread = threading.Thread(target=self._display_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def _display_loop(self):
        while self.running:
            try:
                if not self.queue.empty():
                    obs, episode, step = self.queue.get_nowait()
                    
                    # Create window names
                    rgb_window = f"RGB Camera - Episode {episode}, Step {step}"
                    depth_window = f"Depth Camera - Episode {episode}, Step {step}"
                    
                    # Normalize depth for display
                    depth_normalized = (obs['depth'] / obs['depth'].max() * 255).astype(np.uint8)
                    
                    # Display images
                    cv2.imshow(rgb_window, obs['rgb'])
                    cv2.imshow(depth_window, depth_normalized)
                    cv2.waitKey(1)
                
                time.sleep(0.1)
            except:
                pass
    
    def display_observations(self, obs, episode, step):
        self.queue.put((obs, episode, step))
    
    def stop(self):
        self.running = False
        cv2.destroyAllWindows()

def main():
    print("=" * 60)
    print("UR5 Gripper 3D Simulation + Threaded Camera Display")
    print("=" * 60)
    print("Note: This script must be run with 'mjpython' on macOS")
    print("You should see:")
    print("1. A 3D MuJoCo viewer window showing the UR5 robot")
    print("2. OpenCV windows showing camera observations (in separate thread)")
    print("=" * 60)
    
    # Create camera display thread
    camera_display = CameraDisplayThread()
    
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
        camera_display.display_observations(obs, 0, 0)
        
        # Run a few episodes with both 3D viewer and threaded camera display
        for episode in range(2):  # Reduced for demo
            print(f"\n{'='*50}")
            print(f"EPISODE {episode + 1}")
            print(f"{'='*50}")
            
            obs, info = env.reset()
            camera_display.display_observations(obs, episode + 1, 0)
            
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
                camera_display.display_observations(obs, episode + 1, step + 1)
                
                # Wait to see the action in 3D viewer
                time.sleep(3)
        
        print("\nSimulation complete!")
        print("The 3D viewer and camera windows should still be open.")
        print("Press Ctrl+C to exit...")
        
        # Keep everything open
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nExiting simulation...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        camera_display.stop()
        env.close()
        print("Environment closed.")

if __name__ == "__main__":
    main()
