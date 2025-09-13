#!/usr/bin/env python3
"""
Simple example to show the UR5 gripper in 3D MuJoCo viewer.
This creates a minimal simulation just to see the robot.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import os
from pathlib import Path

def main():
    print("Loading UR5 Gripper 3D Simulation...")
    
    # Get the path to the XML file
    path = os.path.realpath(__file__)
    path = str(Path(path).parent)
    xml_path = path + "/UR5+gripper/UR5gripper_2_finger.xml"
    
    print(f"Loading model from: {xml_path}")
    
    # Load the MuJoCo model
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    print("Model loaded successfully!")
    print(f"Model has {model.nbody} bodies and {model.njnt} joints")
    
    # Launch the viewer
    print("Launching 3D viewer...")
    print("You should see a window with the UR5 robot simulation.")
    print("Controls:")
    print("- Mouse drag: Rotate view")
    print("- Scroll wheel: Zoom")
    print("- Shift + Mouse drag: Pan")
    print("- Close window to exit")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("Viewer launched! The robot should be visible.")
        
        # Run the simulation for a while
        for i in range(1000):
            # Step the simulation
            mujoco.mj_step(model, data)
            
            # Update the viewer
            viewer.sync()
            
            # Small delay to make it visible
            time.sleep(0.01)
            
            # Print progress every 100 steps
            if i % 100 == 0:
                print(f"Simulation step {i}/1000")
    
    print("Simulation complete!")

if __name__ == "__main__":
    main()
