# Dynablox
An online mapping-based approach for real-time dynamic object detection that is agnostic to the object type an environment structure!


# Table of Contents
**Credits**
* [Paper](#Paper)

**Setup**
* [Installation](#Installation)
* [Datasets](#Datasets)

**Examples**
- [Running a DOALS sequence](#Running-a-DOALS-sequence)
- [Running a Dynablox sequence](#Running-a-Dynablox-sequence)
- [Evaluating an Experiment](#Evaluating-an-Experiment)

# Paper
If you find this package useful for your research, please consider citing our paper:

* TODO: Lukas Schmid, Olov Andersson, Aurelio Sulser, Patrick Pfreundschuh, and Roland Siegwart. "**Dynablox: Real-time Detection of Diverse Dynamic Objects in Complex Environments**" in *ArXiv Preprint*, 2023.
  \[ [IEEE](https://ieeexplore.ieee.org/document/9811877) | [ArXiv](https://arxiv.org/abs/2109.10165) | [Video](https://www.youtube.com/watch?v=A7o2Vy7_TV4) \]
  ```bibtex
  @inproceedings{schmid2022panoptic,
    title={Dynablox: Real-time Detection of Diverse Dynamic Objects in Complex Environments},
    author={Schmid, Lukas, and Andersson, Olov, and Sulser, Aurelio, and Pfreundschuh, Patrick, and Siegwart, Roland},
    booktitle={ArXiv Preprint},
    year={2023},
    volume={},
    number={},
    pages={},
    doi={}}
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
    git clone git@github.com:ethz-asl/dynablox.git
    ```

5. Install ROS dependencies:
    ```bash
    # If wstool is not yet initialized:
    wstool init . ./dynablox/ssh.rosinstall 
    # If wstool is already initialized:
    wstool merge -t . ./dynablox/ssh.rosinstall
    # After either case:
    wstool update
    ```

6. Build:
    ```bash
    catkin build dynablox_ros
    ```

## Datasets
To run the demos we use the [Urban Dynamic Objects LiDAR  (DOALS) Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=doals).
To download the data and pre-process it for our demos, use the provided script:
```bash
roscd dynablox_ros/scripts
./download_doals_data.sh <target_path>
```

TODO: We further collect a [new dataset](todo) featuring diverse dynamic objects in complex scenes.
To download the data and pre-process it for our demos, use the provided script:
```bash
roscd dynablox_ros/scripts
./download_dynablox_data.sh <target_path>
```

# Examples
## Running a DOALS Sequence
1. If not done so, download the DOALS dataset as explained [here](#datasets).

2. TODO(schmluk): Adjust some path settings and configs.

3. Run
    ```bash
    roslaunch dynablox_ros run.launch 
    ```


## Running a Dynablox Sequence
1. If not done so, download the dataset as explained [here](#datasets).

2. TODO(schmluk): Adjust some path settings and configs.

3. Run
    ```bash
    roslaunch dynablox_ros run.launch 
    ```


## Evaluating an Experiment

1. If not done so, download the dataset as explained [here](#datasets).

2. TODO(schmluk): Adjust some path settings and configs.

3. Run
    ```bash
    bash scripts/run_experiments.sh
    ```

