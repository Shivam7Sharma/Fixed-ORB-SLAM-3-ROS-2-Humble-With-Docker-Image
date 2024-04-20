# Fixed-ORB-SLAM-3-ROS-2-Humble



This project is an implementation of ORB SLAM 3 in ROS 2. It is a robust and efficient solution for SLAM, leveraging the power of ORB features.

## Dependencies

The project requires the following dependencies to be installed:

- ORB SLAM 3 (Fixed Repo)
- OpenCV 4.2
- ROS 2 Humble
- Gazebo
- Pangolin

## Installation

After installing all the required dependencies, you can build the project using the following command:

```bash
colcon build

## Usage

```
colcon build --symlink-install 
source install/setup.bash
ros2 launch lawn_mower_gazebo lawn_mower_launch.py
```

Here's a detailed README for your project:

```markdown
# Fixed-ORB-SLAM-3-ROS-2-Humble

This project is an implementation of ORB SLAM 3 in ROS 2. It is a robust and efficient solution for SLAM, leveraging the power of ORB features.

## Dependencies

The project requires the following dependencies to be installed:

- ORB SLAM 3 (Fixed Repo)
- OpenCV 4.2
- ROS 2 Humble
- Gazebo
- Pangolin

## Installation

After installing all the required dependencies, you can build the project using the following command:

```bash
colcon build
```

## Usage

To run the simulation, use the following commands:

First delete or empty the build folder of the workspace.

```bash
colcon build --symlink-install 
source install/setup.bash
ros2 launch lawn_mower_gazebo lawn_mower_launch.py
```

If you haven't installed keyboard teleop in ROS 2, please install it to teleoperate the robot with the keyboard.

To run the ORB SLAM 3, use the following command (modify the paths based on your directory names):

```bash
ros2 run orbslam3 stereo /home/shivamss/colcon_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt /home/shivamss/colcon_ws/src/orbslam3_ros2/config/stereo/oakcam.yaml false
```
```

Please replace the paths in the commands with the actual paths in your system.
