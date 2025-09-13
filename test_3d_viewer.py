#!/usr/bin/env python3
"""
Simple test to verify the 3D viewer works with the gymnasium environment.
Run with: mjpython test_3d_viewer.py
"""

import gymnasium as gym
import time

def main():
    print("Testing 3D viewer with gymnasium environment...")
    print("This should show a 3D MuJoCo viewer window with the UR5 robot.")
    
    # Create environment with rendering enabled
    env = gym.make("gym_grasper:Grasper-v0", show_obs=False, render=True)
    
    print("Environment created! You should see a 3D viewer window.")
    
    try:
        # Reset the environment
        obs, info = env.reset()
        print("Environment reset. Robot should be visible in the 3D viewer.")
        
        # Wait a bit to see the robot
        print("Waiting 5 seconds to see the robot...")
        time.sleep(5)
        
        # Run a simple action
        action = env.action_space.sample()
        print(f"Running action: {action}")
        
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"Action completed. Reward: {reward}")
        
        # Wait a bit more
        print("Waiting 5 more seconds...")
        time.sleep(5)
        
        print("Test complete! Close the 3D viewer window to exit.")
        
        # Keep the viewer open
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        env.close()

if __name__ == "__main__":
    main()
