from setuptools import setup, find_packages

setup(
    name="gym_grasper",
    version="0.0.1",
    packages=find_packages(),
    install_requires=["gymnasium", "mujoco", "numpy", "simple_pid", "termcolor", "ikpy", "pyquaternion", "opencv-python", "matplotlib"],
)
