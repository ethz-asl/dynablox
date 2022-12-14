# ROS Package for Moving Object Detection in 3D LiDAR data




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
