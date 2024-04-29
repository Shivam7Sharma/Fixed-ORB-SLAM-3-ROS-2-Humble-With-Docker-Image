# Fixed-ORB-SLAM-3-ROS-2-Humble


This project is an implementation of ORB SLAM 3 in ROS 2. It is a robust and efficient solution for SLAM, leveraging the power of ORB features.

## Dependencies

The project requires the following dependencies to be installed:

- ORB SLAM 3 (Fixed Repo)
- OpenCV 4.2
- ROS 2 Humble
- Gazebo
- Pangolin
## Docker Image
```docker pull shivam157/orb_slam3_custom```
ROS 2 workspace /Fixed-ORB-SLAM-3-ROS-2-Humble

command for stereo:
``` ros2 run orbslam3 stereo /Fixed-ORB-SLAM-3-ROS-2-Humble/src/orbslam3_ros2/vocabulary/ORBvoc.txt /Fixed-ORB-SLAM-3-ROS-2-Humble/src/orbslam3_ros2/config/stereo/oakcam.yaml false ```

## Fixes
The image and the repo might need some fixes for the ros 2 inertial, mono, and rgbd nodes. See link:
[Issue #16 on ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2/issues/16)


## Installation

After installing all the required dependencies, you can build the project using the following command:

``` colcon build ```

## Usage

```
colcon build --symlink-install 
source install/setup.bash
ros2 launch lawn_mower_gazebo lawn_mower_launch.py
```

If you haven't installed keyboard teleop in ROS 2, please install it to teleoperate the robot with the keyboard.

To run the ORB SLAM 3, use the following command (modify the paths based on your directory names):

```
ros2 run orbslam3 stereo /home/shivamss/colcon_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt /home/shivamss/colcon_ws/src/orbslam3_ros2/config/stereo/oakcam.yaml false
```

Please replace the paths in the commands with the actual paths in your system.
