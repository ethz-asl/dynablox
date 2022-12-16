# ROS Package for Moving Object Detection in 3D LiDAR data




## Installation

* **Note on Versioning:** This package was developed using Ubuntu 20.04 using ROS Noetic. Other versions should also work but support can not be guaranteed.

1. If not already done so, install [ROS](http://wiki.ros.org/action/fullsearch/melodic/Installation/Ubuntu?action=fullsearch&context=180&value=linkto%3A%22melodic%2FInstallation%2FUbuntu%22). We recommend using `Desktop-Full`.

2. If not already done so, setup a catkin workspace:
  ```
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws
  catkin init
  catkin config --extend /opt/ros/$ROS_DISTRO
  catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
  catkin config --merge-devel
  ```

3. Install system dependencies:
  ```
  sudo apt-get install python-wstool python-catkin-tools ros-$ROS_DISTRO-cmake-modules protobuf-compiler autoconf
  ```

4. Clone the repo using [SSH Keys](https://docs.github.com/en/authentication/connecting-to-github-with-ssh):
  ```
  cd ~/catkin_ws/src
  git clone git@github.com:ethz-asl/lidar_motion_detection.git
  ```

5. Install ROS dependencies:
  ```
  wstool init . ./lidar_motion_detectionlox/ssh.rosinstall # if wstool is not yet initialized
  wstool merge -t . ./lidar_motion_detectionlox/ssh.rosinstall # if wstool is not yet initialized
  wstool update
  ```

6. Build:
  ```
  catkin build lidar_motion_detectionlox
  ```

# Old

This repository contains a ROS package that can be used for moving object detection and tracking.
The pipeline is based on online volumetric mapping and relies on [Voxblox](https://github.com/ethz-asl/voxblox).
The three main processes or levels of the pipeline are the following:
1. Classifying every point of a pointcloud as either static or dynamic using a fusion of different motion cues.
2. Spatial clustering of candidate dynamic points and cluster validation.
3. Tracking moving objects over time.

![Pipeline Overview](images/overview_2.png)

The pipeline differs from other approaches through a combination of the following characteristics:
- No restrictions to classes of detected moving objects.
- No assumtion on a robot's environment.
- Runs on a CPU.

<img src="/images/staircase_3_16_9.png" width="180"/> <img src="/images/ball_1_16_9.png" width="180"/> <img src="/images/trolley_3_16_9.png" width="180"/> <img src="/images/stairs_d_16_9.png" width="180"/> 



# Demo Usage:

1. Clone the repository to your catkin workspace, install the necessary dependencies & build the package. In case of [Voxblox](https://github.com/ethz-asl/voxblox), the package needs [this specific branch](https://github.com/samuel-gull/voxblox/tree/feature/lidar_motion_detection).


2. Download one of the real world sequences from the [Urban Dynamic Objects LiDAR Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=doals).

3. Run the following command:
```
roslaunch lidar_motion_detection demo.launch bag_file:="path to bag file"
``` 
