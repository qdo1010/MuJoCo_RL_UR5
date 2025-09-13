## Accompanying repository of Master's thesis at TU Berlin / Aalborg University. **MIGRATED TO LATEST MUJOCO** - Now compatible with Apple Silicon (M1/M2/M3/M4) MacBooks!

# Deep Reinforcement Learning for robotic pick and place applications using purely visual observations 
**Orginal Author:** Paul Daniel (paudan22@gmail.com)  
**Migration:** Updated from mujoco-py to DeepMind MuJoCo + Gymnasium for modern compatibility

### Traits of this environment: Very large and multi-discrete actionspace, very high sample-cost, visual observations, binary reward.

## 🚀 **MAJOR UPDATE: Apple Silicon Compatible!**

This repository has been **completely migrated** from the deprecated `mujoco-py` to the latest **DeepMind MuJoCo** library, making it fully compatible with:
- ✅ **Apple Silicon MacBooks** (M1, M2, M3, M4)
- ✅ **Latest Python versions** (3.8+)
- ✅ **Modern Gymnasium API** (replacing deprecated gym)
- ✅ **Cross-platform compatibility** (Windows, macOS, Linux)

### What Changed:
- **MuJoCo**: `mujoco-py` → `mujoco` (DeepMind's official library)
- **Gym**: `gym` → `gymnasium` (modern Gym API)
- **Viewer**: Updated to use MuJoCo's native viewer
- **API**: All MuJoCo API calls updated to latest syntax
- **Dependencies**: Updated all packages to latest versions


Trained agent in action            |  Example of predicted grasp chances 
:---------------------------------:|:-------------------------:
![](/media/gif_3.gif "Trained Agent")  |  ![](/media/gif_4.gif "Overlay")


Setup iteration            |  Relevant changes 
:-------------------------:|:-------------------------:
  IT5 | - Many more objects, randomly piled <br> - Actionspace now multi-discrete, with second dimension being a rotation action
  IT4 | - Z-coordinate for grasping now calculated using depth data <br> - Objects now vary in size
  IT3 | - New two-finger gripper implemented 
  IT2 | - Grasp success check now after moving to drop location (1000 steps)
  IT1 (Baseline)|- Grasp success check after moving straight up (500 steps of trying to close the gripper) <br> - Fixed z-coordinate for grasping <br> - Objects of equal size


This repository provides several python classes for control of robotic arms in MuJoCo: 

 * **MJ_Controller:** This class can be used as a standalone class for basic robot control in MuJoCo. This can be useful for trying out models and their grasping capabilities. 
 Alternatively, its methods can also be used by any other class (like a Gym environment) to provide some more functionality. One example of this might be to move the robot back into a certain position after every episode of training, which might be preferable compared to just resetting all the joint angles and velocities. 
 The controller currently also holds the methods for image transformations, which might be put into another separate class at some point. 

* **GraspEnv:** A Gym environment for training reinforcement learning agents. The task to master is a pick & place task. 
The difference to most other MuJoCo Gym environments is that the observation returned is a camera image instead of a state vector of the simulation. This is meant to resemble a real world setup more closely. 

The robot configuration used in this setup (Universal Robots UR5 + Robotiq S Model 3 Finger Gripper) is based on [this](http://www.mujoco.org/forum/index.php?resources/universal-robots-ur5-robotiq-s-model-3-finger-gripper.22/) resource. It has since been heavily modified. Most current XML-file: *UR5gripper_2_finger.xml*  

**Updated Dependencies:**
- **MuJoCo**: Now uses [DeepMind MuJoCo](https://github.com/deepmind/mujoco) (official library)
- **Gymnasium**: Modern replacement for OpenAI Gym
- **PID controllers**: Based on [simple_pid](https://github.com/m-lundberg/simple-pid)
- **Inverse kinematics**: Using [ikpy](https://github.com/Phylliade/ikpy)
- **Image processing**: OpenCV for camera observations
- **Visualization**: Matplotlib for plotting

## **🚀 Quick Setup (Recommended)**

### **Option 1: Virtual Environment (Recommended)**

```bash
# Clone the repository
git clone https://github.com/PaulDanielML/MuJoCo_RL_UR5.git
cd MuJoCo_RL_UR5/

# Create and activate virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Install the package in development mode
pip install -e .
```

### **Option 2: Global Installation (Not Recommended)**

```bash
# Clone the repository
git clone https://github.com/qdo1010/MuJoCo_RL_UR5.git
cd MuJoCo_RL_UR5/

# Install dependencies
pip install -r requirements.txt

# Install the package
pip install -e .
```

### **🔧 System Requirements**

- **Python**: 3.8 or higher
- **Operating System**: Windows, macOS (Intel & Apple Silicon), Linux
- **Memory**: At least 4GB RAM recommended
- **Graphics**: OpenGL support for 3D rendering

### **📦 Dependencies**

The following packages are automatically installed via `requirements.txt`:
- `mujoco` - DeepMind's official MuJoCo library
- `gymnasium` - Modern Gym API
- `numpy` - Numerical computing
- `opencv-python` - Computer vision
- `matplotlib` - Plotting and visualization
- `simple-pid` - PID controllers
- `ikpy` - Inverse kinematics
- `pyquaternion` - Quaternion operations
- `termcolor` - Colored terminal output

### **🍎 macOS Specific Notes**

On macOS, you need to use `mjpython` instead of `python` to run scripts with the 3D viewer:

```bash
# For scripts with 3D viewer
mjpython example_3d_only.py

# For scripts without 3D viewer
python example_agent.py
```

### **🔍 Verification**

Test your installation:

```bash
# Test basic functionality
python example_agent.py

# Test 3D viewer (macOS)
mjpython example_3d_only.py

# Test camera observations
python example_camera_only.py
```  



## **📚 Example Scripts**

The repository now includes multiple example scripts to demonstrate different features:

### **🎮 Basic Examples**
- **`example_agent.py`** - Basic random agent with camera observations
- **`example.py`** - Simple controller demonstration
- **`simple_3d_viewer.py`** - Minimal 3D viewer example

### **🤖 3D Visualization Examples**
- **`example_3d_only.py`** - 3D MuJoCo viewer only (recommended for macOS)
- **`example_3d_gymnasium.py`** - Gymnasium environment with 3D viewer
- **`example_3d_with_saved_camera.py`** - 3D viewer + camera images saved to files
- **`example_3d_with_matplotlib.py`** - 3D viewer + matplotlib camera display
- **`example_3d_with_threaded_camera.py`** - 3D viewer + threaded OpenCV camera display

### **📷 Camera Observation Examples**
- **`example_camera_only.py`** - Camera observations only (no 3D viewer)
- **`example_agent_with_3d.py`** - Agent with both 3D viewer and camera

### **🔧 Advanced Examples**
- **`Grasping_Agent_multidiscrete.py`** - Multi-discrete action space agent
- **`normalize.py`** - Image normalization utility

### **📁 Offline RL Examples**
- **`Offline RL/generate_data.py`** - Generate training data
- **`Offline RL/train.py`** - Train offline RL agents
- **`Offline RL/grasping_dataset.py`** - Dataset utilities

## **Usage**

### **GraspEnv - class:**

Gymnasium environment for training agents to use RGB-D data for predicting pixel-wise grasp success chances.  
The file [*example_agent.py*](example_agent.py) demonstrates the use of a random agent for this environment.  
The file [*Grasping_Agent_multidiscrete.py*](Grasping_Agent_multidiscrete.py) gives an example of training a shortsighted DQN-agent in the environment to predict pixel-wise grasping success (PyTorch).
The created environment has an associated controller object, which provides all the functionality of the *MJ_Controller* - class to it. 

* **Action space**: Pixel space, can be specified by setting height and width. Current defaults: 200x200. This means there are 40.000 possible actions. This resolution translates to a picking accuracy of ~ 4mm.
* **State space**: The states / observations provided are dictionaries containing two arrays: An RGB-image and a depth-image, both of the same resolution as the action space
* **Reward function**: The environment has been updated to a binary reward structure:
    * 0 for choosing a pixel that does not lead to a successful grasp.
    * +1 for choosing a pixel that leads to a successful grasp.

The user gets a summary of each step performed in the console. It is recommended to train agents without rendering, as this will speed up training significantly. 
    
![console](/media/console.png "Example console output during training")

The rgb part of the last captured observation will be shown and updated in an extra window.

![observation](/media/observation_rgb.png "Example observation")

### **🔧 Troubleshooting**

#### **Common Issues:**

1. **"Unknown C++ exception from OpenCV code" on macOS**
   - **Solution**: Use separate scripts for 3D viewer and camera observations
   - **3D Viewer**: `mjpython example_3d_only.py`
   - **Camera Only**: `python example_camera_only.py`
   - **Both (saved)**: `mjpython example_3d_with_saved_camera.py`

2. **"launch_passive requires mjpython on macOS"**
   - **Solution**: Use `mjpython` instead of `python` for scripts with 3D viewer
   - **Example**: `mjpython example_3d_only.py`

3. **"ModuleNotFoundError: No module named 'mujoco'"**
   - **Solution**: Install dependencies: `pip install -r requirements.txt`

4. **"ModuleNotFoundError: No module named 'gymnasium'"**
   - **Solution**: Install gymnasium: `pip install gymnasium`

5. **Performance Issues**
   - **Solution**: Disable rendering during training: `render=False`
   - **Solution**: Use `show_obs=False` to disable camera windows

#### **Platform-Specific Notes:**

- **macOS**: Use `mjpython` for 3D viewer, `python` for camera-only scripts
- **Windows/Linux**: Use `python` for all scripts
- **Apple Silicon**: Fully supported with the new MuJoCo library

### **MJ_Controller - class:**

Example usage of some of the class methods is demonstrated in the file [*example.py*](example.py).

The class *MJ_Controller* offers high and low level methods for controlling the robot in MuJoCo. 

* **move_ee** : High level, moves the endeffector of the arm to the desired XYZ position (in world 					coordinates). This is done using very simple inverse kinematics, just obeying the joint limits. Currently there is not collision avoidance implemented. Since this whole repo is created with grasping in mind, the delivered pose will always be so that the gripper is oriented in a vertical way (for top down grasps).
* **actuate_joint_group** :  Low level, lets the user specify motor activations for a specified group
* **grasp** : Uses the specified gripper group to attempt a grasp. A simple check is done to determine weather the grasp was successful or not and the result will be output blinking in the console. 

![gif1](/media/gif_1.gif "Simple Grasp and Toss")

## **🔄 Migration Details**

### **What Was Changed:**

#### **Core Libraries:**
- **`mujoco-py` → `mujoco`**: Complete migration to DeepMind's official MuJoCo library
- **`gym` → `gymnasium`**: Updated to modern Gym API with proper environment wrapping
- **API Updates**: All MuJoCo API calls updated to latest syntax

#### **Key File Changes:**
- **`gym_grasper/controller/MujocoController.py`**: Complete refactor for new MuJoCo API
- **`gym_grasper/envs/GraspingEnv.py`**: Migrated from `gym.envs.mujoco` to `gymnasium.Env`
- **`requirements.txt`**: Updated all dependencies to latest versions
- **`setup.py`**: Updated package configuration for modern Python

#### **New Features Added:**
- **Multiple 3D Viewer Examples**: Various approaches to show 3D simulation
- **Camera Observation Examples**: Separate scripts for camera-only viewing
- **Cross-Platform Compatibility**: Works on Windows, macOS (Intel & Apple Silicon), Linux
- **Modern Environment API**: Proper Gymnasium environment with `reset()` and `step()` methods

### **Breaking Changes:**
- **Environment Creation**: Now uses `gymnasium.make()` instead of `gym.make()`
- **Reset Method**: Returns `(observation, info)` tuple instead of just observation
- **Step Method**: Returns 5 values `(obs, reward, terminated, truncated, info)` instead of 4
- **Viewer**: Uses MuJoCo's native viewer instead of mujoco-py's viewer

## **📈 Updates & Features**

**🚀 Major Migration (2024):** Complete migration from mujoco-py to DeepMind MuJoCo for Apple Silicon compatibility

**Trials for Offline RL:** The folder *Offline RL* contains scripts for generating and learning from a dataset of (state, action, reward)-transitions. *generate_data.py* can be used to generate as many files as required, each file containing 12 transitions.

**New gripper model available:** A new, less bulky, 2-finger gripper was implemented in the model in training setup iteration 3. 

![new_gripper](/media/new_gripper.png "new gripper")

**Image normalization:** Added script *normalize.py*, which samples 100 images from the environment and writes the mean values and standard deviations of all channels to a file. 

**Reset shuffle:** Calling the environments *step* method now rearranges all the pickable objects to random positions on the table. 

![gif2](/media/gif_2.gif "Respawning")

**Record grasps:** The step method of the *GraspingEnv* now has the optional parameter *record_grasps*. If set to True, it will capture a side camera image every time a grasp is made that is deemed successful by the environment. This allows for "quality control" of the grasps, without having to watch all the failed attempts. The captured images can also be useful for fine tuning grasping parameters. 

![grasp](/media/grasp.png "Example grasp")

**Point clouds:** The controller class was provided with new methods for image transformations. 
* depth_2_meters: Converts the normalized depth values returned by MuJoCo into meters.
* create_camera_data: Constructs a camera matrix, focal length and sets the camera's position and rotation based on a provided camera name and desired image width and depth. 
* world_2_pixel: Accepts a XYZ world position and returns the corresponding x-y pixel coordinates 
* pixel_2_world: Accepts x-y pixel coordinates and a depth value, returns the XYZ world position. This method can be used to construct point clouds out of the data returned by the controllers *get_image_data* method.

![cloud](/media/point_cloud.png "Example point cloud")

**Joint plots:** All methods that move joints now have an optional *plot* parameter. If set to True, a .png-file will be created in the local directory. It will show plots for each joint involved in the trajectory, containing the joint angles over time, as well as the target values. This can be used to determine which joints overshoot, oscillate etc. and adjust the controller gains based on that.  
The tolerance used for the trajectory are plotted in red, so it can easily be determined how many steps each of the joints needs to reach a value within tolerance. 

![plot1](/media/plot_1.png "Example plot")

## **🤝 Contributing**

This repository was migrated to support modern hardware and software. If you encounter issues or have improvements:

1. **Check the troubleshooting section** above
2. **Test with the provided example scripts**
3. **Report issues** with your system specifications
4. **Contribute fixes** for cross-platform compatibility

## **📄 License**

This project maintains the original license. See `license.txt` for details.

## **🙏 Acknowledgments**

- **Original Author**: Paul Daniel (paudan22@gmail.com)
- **Migration**: Updated for modern MuJoCo and Apple Silicon compatibility
- **Robot Model**: Based on Universal Robots UR5 + Robotiq gripper from MuJoCo forums
- **Dependencies**: Thanks to all the open-source libraries that make this possible
