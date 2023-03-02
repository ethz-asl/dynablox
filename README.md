# ROS Package for Moving Object Detection in 3D LiDAR data


# Table of Contents
**Credits**
* [Paper](#Paper)

**Setup**
* [Installation](#Installation)
* [Datasets](#Datasets)

**Examples**
- [Running a DOALS sequence](#Running-a-DOALS-sequence)
- [Processing the DOALS dataset](#Processing-the-DOALS-dataset)


# Paper
If you find this package useful for your research, please consider citing our paper:

* TODO: Authors. "**Title**" in *Venue*, pp. 8018-8024, 2022.
  \[ [IEEE](https://ieeexplore.ieee.org/document/9811877) | [ArXiv](https://arxiv.org/abs/2109.10165) | [Video](https://www.youtube.com/watch?v=A7o2Vy7_TV4) \]
  ```bibtex
  @inproceedings{schmid2022panoptic,
    title={Panoptic Multi-TSDFs: a Flexible Representation for Online Multi-resolution Volumetric Mapping and Long-term Dynamic Scene Consistency},
    author={Schmid, Lukas and Delmerico, Jeffrey and Sch{\"o}nberger, Johannes and Nieto, Juan and Pollefeys, Marc and Siegwart, Roland and Cadena, Cesar},
    booktitle={2022 IEEE International Conference on Robotics and Automation (ICRA)},
    year={2022},
    volume={},
    number={},
    pages={8018-8024},
    doi={10.1109/ICRA46639.2022.9811877}}
  }
  ```

# Setup
## Installation

* **Note on Versioning:** This package was developed using Ubuntu 20.04 using ROS Noetic. Other versions should also work but support can not be guaranteed.

1. If not already done so, install [ROS](http://wiki.ros.org/action/fullsearch/melodic/Installation/Ubuntu?action=fullsearch&context=180&value=linkto%3A%22melodic%2FInstallation%2FUbuntu%22). We recommend using `Desktop-Full`.

2. If not already done so, setup a catkin workspace:
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin init
    catkin config --extend /opt/ros/$ROS_DISTRO
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    catkin config --merge-devel
    ```

3. Install system dependencies:
    ```bash
    sudo apt-get install python3-wstool python3-catkin-tools ros-$ROS_DISTRO-cmake-modules protobuf-compiler autoconf libjsoncpp-dev libspdlog-dev
    ```

4. Clone the repo using [SSH Keys](https://docs.github.com/en/authentication/connecting-to-github-with-ssh):
    ```bash
    cd ~/catkin_ws/src
    git clone git@github.com:ethz-asl/lidar_motion_detection.git
    ```

5. Install ROS dependencies:
    ```bash
    # If wstool is not yet initialized:
    wstool init . ./lidar_motion_detection/ssh.rosinstall 
    # If wstool is already initialized:
    wstool merge -t . ./lidar_motion_detection/ssh.rosinstall
    # After either case:
    wstool update
    ```

6. Build:
    ```bash
    catkin build lidar_motion_detection_ros
    ```

## Datasets
To run the demos we use the [Urban Dynamic Objects LiDAR  (DOALS) Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=doals).
To download the data and pre-process it for our demos, use the provided script:
```bash
roscd lidar_motion_detection_ros/scripts
chmod +x download_doals.sh
./download_doals.sh <target_path>
```

# Examples
## Running a DOALS sequence
1. If not done so, download the dataset as explained [here](#datasets).

2. TODO(schmluk): Adjust some path settings and configs.

3. Run
    ```bash
    roslaunch lidar_motion_detection_ros run.launch 
    ```

## Processing the DOALS dataset

1. If not done so, download the dataset as explained [here](#datasets).

2. TODO(schmluk): Adjust some path settings and configs.

3. Run
    ```bash
    bash scripts/run_experiments.sh
    ```

