![Ubuntu 20.04 + ROS Noetic: Build](https://github.com/ethz-asl/dynablox/actions/workflows/build_test_20.yml/badge.svg)

# Dynablox
An online volumetric mapping-based approach for real-time detection of diverse dynamic objects in complex environments.


<p align="center">
  <img width='100%' src="https://user-images.githubusercontent.com/36043993/232650770-a042cbb9-c251-42f9-8a96-d7d9273fec96.gif">
</p>

# Table of Contents
**Credits**
* [Paper](#Paper)

**Setup**
* [Installation](#Installation)
* [Datasets](#Datasets)

**Examples**
- [Running a DOALS sequence](#Running-a-DOALS-sequence)
- [Running a Dynablox sequence](#Running-a-Dynablox-sequence)
- [Running and Evaluating an Experiment](#Evaluating-an-Experiment)

# Paper
If you find this package useful for your research, please consider citing our paper:

* Lukas Schmid, Olov Andersson, Aurelio Sulser, Patrick Pfreundschuh, and Roland Siegwart. "**Dynablox: Real-time Detection of Diverse Dynamic Objects in Complex Environments**" in *ArXiv Preprint*, 2023. \[ [ArXiv](https://arxiv.org/abs/2304.10049) \]
  ```bibtex
  @inproceedings{schmid2023dynablox,
    title={Dynablox: Real-time Detection of Diverse Dynamic Objects in Complex Environments},
    author={Schmid, Lukas, and Andersson, Olov, and Sulser, Aurelio, and Pfreundschuh, Patrick, and Siegwart, Roland},
    booktitle={ArXiv Preprint 2304.10049},
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

2. Install system dependencies:
    ```bash
    sudo apt-get install python3-vcstool python3-catkin-tools ros-$ROS_DISTRO-cmake-modules protobuf-compiler autoconf git rsync -y   
    ```

3. Clone the repo using [SSH Keys](https://docs.github.com/en/authentication/connecting-to-github-with-ssh):
    ```bash
    cd ~/catkin_ws/src
    git clone git@github.com:ethz-asl/dynablox.git
    ```

4. Install ROS dependencies:
    ```bash
    cd ~/catkin_ws/src
    vcs import . < ./dynablox/ssh.rosinstall --recursive 
    ```

5. Build:
    ```bash
    catkin build dynablox_ros
    ```

## Datasets
To run the demos we use the [Urban Dynamic Objects LiDAR  (DOALS) Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=doals).
To download the data and pre-process it for our demos, use the provided script:
```bash
roscd dynablox_ros/scripts
# Or your preferred data destination.
./download_doals_data.sh /home/$USER/data/DOALS
```

> __note__ The dataset will be released shortly!

We further collect a new dataset featuring diverse dynamic objects in complex scenes.
To download the processed ready-to-run data for our demos, use the provided script:
```bash
roscd dynablox_ros/scripts
# Or your preferred data destination.
./download_dynablox_data.sh /home/$USER/data/Dynablox
```

# Examples
## Running a DOALS Sequence
1. If not done so, download the DOALS dataset as explained [here](#datasets).

2. Adjust the dataset path in `dynablox_ros/launch/run_experiment.launch`:
    ```xml
    <arg name="bag_file" default="/home/$(env USER)/data/DOALS/hauptgebaeude/sequence_1/bag.bag" />  
    ```
3. Run
    ```bash
    roslaunch dynablox_ros run_experiment.launch 
    ```
4. You should now see dynamic objects being detected as the sensor moves through the scene:

![Run DOALS Example](https://user-images.githubusercontent.com/36043993/232138501-84250c43-236e-46f6-9b50-af54312215a7.png)

## Running a Dynablox Sequence
> __note__ The dataset will be released shortly!

1. If not done so, download the Dynablox dataset as explained [here](#datasets).

2. Adjust the dataset path in `dynablox_ros/launch/run_experiment.launch` and set `use_doals` to false:
    ```xml
    <arg name="use_doals" default="false" /> 
    <arg name="bag_file" default="/home/$(env USER)/data/Dynablox/processed/ramp_1.bag" />  
    ```
3. Run
    ```bash
    roslaunch dynablox_ros run_experiment.launch 
    ```
4. You should now see dynamic objects being detected as the sensor moves through the scene:
![Run Dynablox Example](https://user-images.githubusercontent.com/36043993/232140093-ee99a919-d2ad-4dc8-95ac-fa047b901f94.png)


## Running and Evaluating an Experiment

### Running an Experiment

1. If not done so, download the DOALS dataset as explained [here](#datasets).

2. Adjust the dataset path in `dynablox_ros/launch/run_experiment.launch`:
    ```xml
    <arg name="bag_file" default="/home/$(env USER)/data/DOALS/hauptgebaeude/sequence_1/bag.bag" />  
    ```

3. In `dynablox_ros/launch/run_experiment.launch`, set the `evaluate` flag, adjust the ground truth data path, and specify where to store the generated outpuit data:
    ```xml
    <arg name="evaluate" default="true" />
    <arg name="eval_output_path" default="/home/$(env USER)/dynablox_output/" />
    <arg name="ground_truth_file" default="/home/$(env USER)/data/DOALS/hauptgebaeude/sequence_1/indices.csv" />
      ```
3. Run
    ```bash
    roslaunch dynablox_ros run_experiment.launch 
    ```

4. Wait till the dataset finished processing. Dynablox should shutdown automatically afterwards.

### Analyzing the Data
- **Printing the Detection Performance Metrics:** 
    1. Run:
    ```bash
    roscd dynablox_ros/src/evaluation
    python3 evaluate_data.py /home/$USER/dynablox_output
    ```
    2. You should now see the performance statistics for all experiments in that folder:
    ```
    1/1 data entries are complete.
    Data                     object_IoU               object_Precision              object_Recall
    hauptgebaeude_1          89.8 +- 5.6              99.3 +- 0.4                   90.3 +- 5.6
    All                      89.8 +- 5.6              99.3 +- 0.4                   90.3 +- 5.6
    ```

- **Inspecting the Segmentation:**
    1. Run:
    ```bash
    roslaunch dynablox_ros cloud_visualizer.launch file_path:=/home/$USER/dynablox_output/clouds.csv
    ```
    2. You should now see the segmentation for the annotated ground truth clouds, showing True Positives (green), True Negatives (black), False Positives (blue), False Negatives (red), and out-of-range (gray) points:
    ![Evaluation](https://user-images.githubusercontent.com/36043993/232151598-750a6860-e6e6-44bc-89c6-fbc866109019.png)

- **Inspecting the Run-time and Configuration:**
    Additional information is automatically stored in `timings.txt` and `config.txt` for each experiment.

### Advanced Options
* **Adding Drift to an Experiment:**
    To run an experiment with drift specify one of the pre-computed drift rollouts in `dynablox_ros/launch/run_experiment.launch`:
    ```xml
    <arg name="drift_simulation_rollout" default="doals/hauptgebaeude/sequence_1/light_3.csv" />
    ```
    All pre-computed rollouts can be found in `drift_simulation/config/rollouts`. Note that the specified sequence needs to match the data being played. For each sequence, there exist 3 rollouts for each intensity.

    Alternatively, use the `drift_simulation/launch/generate_drift_rollout.launch` to create new rollouts for other datasets.

* **Changing th Configuration of Dynablox:**
    All parameters that exist in dynablox are listed in `dynablox_ros/config/motion_detector/default.yaml`, feel free to tune the method for your use case!

