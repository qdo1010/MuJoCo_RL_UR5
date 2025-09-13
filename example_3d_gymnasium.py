#!/usr/bin/env python3
"""
Example to show the UR5 gripper simulation using Gymnasium with 3D viewer.
This script must be run with mjpython on macOS.
"""

import gymnasium as gym
import numpy as np
import time
from termcolor import colored

def main():
    print("=" * 60)
    print("UR5 Gripper 3D Simulation with Gymnasium")
    print("=" * 60)
    print("Note: This script must be run with 'mjpython' on macOS")
    print("=" * 60)
    
    # Create environment with rendering enabled
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
        
        # Show some information about the robot
        print("\nRobot Information:")
        print(f"Model has {unwrapped_env.model.nbody} bodies")
        print(f"Model has {unwrapped_env.model.njnt} joints")
        print(f"Model has {len(unwrapped_env.data.ctrl)} actuators")
        
        # Demonstrate some robot movements
        print("\nDemonstrating robot movements...")
        
        # Move to a few different positions
        positions = [
            [0.0, -0.6, 1.1],    # Center position
            [0.2, -0.4, 1.0],    # Right side
            [-0.2, -0.4, 1.0],   # Left side
            [0.0, -0.6, 1.1],    # Back to center
        ]
        
        for i, pos in enumerate(positions):
            print(f"Moving to position {i+1}: {pos}")
            
            # Move the end-effector to the specified position
            result = unwrapped_env.controller.move_ee(
                pos, 
                max_steps=1000, 
                tolerance=0.05, 
                render=True,  # This will update the viewer
                quiet=False
            )
            
            print(f"Movement result: {result}")
            
            # Wait a bit to see the movement
            time.sleep(2)
        
        # Demonstrate gripper opening and closing
        print("\nDemonstrating gripper movements...")
        
        print("Opening gripper...")
        unwrapped_env.controller.open_gripper(render=True, quiet=False)
        time.sleep(2)
        
        print("Closing gripper...")
        unwrapped_env.controller.close_gripper(render=True, quiet=False)
        time.sleep(2)
        
        print("Opening gripper again...")
        unwrapped_env.controller.open_gripper(render=True, quiet=False)
        time.sleep(2)
        
        # Run a few random actions to see the robot in action
        print("\nRunning random actions...")
        for step in range(3):
            print(f"\nStep {step + 1}:")
            action = env.action_space.sample()
            print(f"Action: {action}")
            
            # Execute the action
            obs, reward, terminated, truncated, info = env.unwrapped.step(
                action, 
                record_grasps=False, 
                markers=False
            )
            
            print(f"Reward: {reward}")
            time.sleep(3)  # Wait to see the action
        
        print("\nSimulation complete!")
        print("The 3D viewer should still be open. You can:")
        print("- Rotate the view by dragging with the mouse")
        print("- Zoom with the scroll wheel")
        print("- Pan by holding Shift and dragging")
        print("- Close the viewer window when done")
        
        # Keep the viewer open
        print("\nKeeping viewer open. Press Ctrl+C to exit...")
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
