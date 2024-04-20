# Fixed-ORB-SLAM-3-ROS-2-Humble

This repository contains an implementation of ORB SLAM 3 for ROS 2 Humble. It provides a robust and efficient solution for Simultaneous Localization and Mapping (SLAM) by leveraging the power of ORB features.

## Dependencies

Before installing the project, ensure the following dependencies are installed:

- **ORB SLAM 3 (Fixed Repo)**: Ensure you use the fixed version compatible with ROS 2.
- **OpenCV 4.2**: Required for image processing functionalities.
- **ROS 2 Humble**: The ROS 2 release targeted by this implementation.
- **Gazebo**: Used for running simulations.
- **Pangolin**: Required for visualization purposes.

## Installation

To build the project, follow these steps after installing all required dependencies:
```colcon build```
## Usage
To use this project, perform the following steps:

Build the project:
Ensure you first delete or empty the build folder of your workspace, then execute:
```colcon build --symlink-install``` 
Set up the environment:
Load the necessary environment variables:
```source install/setup.bash```
Launch the simulation:
Start the ROS 2 simulation with:

``` ros2 launch lawn_mower_gazebo lawn_mower_launch.py ```
If you haven't installed the keyboard teleoperation package in ROS 2, please install it to control the robot using your keyboard.
Run ORB SLAM 3:
To execute ORB SLAM 3, use the following command, modifying paths as necessary based on your directory structure:

``` ros2 run orbslam3 stereo /path/to/ORBvoc.txt /path/to/oakcam.yaml false ```
Replace /path/to/ORBvoc.txt and /path/to/oakcam.yaml with the actual paths to your ORB vocabulary file and camera configuration file, respectively.
Ensure all paths in the commands are updated to reflect your actual file system structure.
