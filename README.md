# Multi-Level-Adaptation-for-Automatic-Landing-with-Engine-Failure-under-Weather-Uncertainties

This repository contains code for ouster_64 beams, with support of a lightweight and ground optimized lidar odometry and mapping (LeGO-LOAM) system for ROS compatible UGVs. The system takes in point cloud  from a Ouster 64-beams Lidar (palced horizontally) and optional IMU data as inputs. It outputs 6D pose estimation in real-time. A demonstration of the system can be found here -> https://www.youtube.com/watch?v=2c1hPMZSF4I
<!--
[![Watch the video](/LeGO-LOAM/launch/demo.gif)](https://www.youtube.com/watch?v=2c1hPMZSF4I)
-->

This picture blow is ouster_data_set displayed in Open_GL and Rviz seperately.
<p align='center'>
    <img src="/LeGO-LOAM/launch/ouster_data_set.png" alt="drawing" width="400"/>
</p>

<p align='center'>
    <img src="/LeGO-LOAM/launch/ouster_sata_set_Rviz.png" alt="drawing" width="400"/>
</p>

## Lidar-inertial Odometry

An updated lidar-initial odometry package, [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM), has been open-sourced and available for testing.

## Dependency

- [ROS](http://wiki.ros.org/ROS/Installation) (tested with indigo, kinetic, and melodic)
- [Ouster_Sample_Drive](optional)(https://github.com/haotiangu1993/ouster_sample_drive.git)


   You can follow up the build rule to complie ouster sample drive. When I worked 
   on this project, the recomended sample data set contained not standard point cloud.
   In order to make sure the LeGO_LOAM can subscribe the point cloud in dataset, I 
   used ouster sample drive to convert the PacketMsg type to PCL Type and displayed 
   it in Rviz. But it seems like recently updated sample data set in official website
   contained point cloud in PCL type which Lego_LOAM can subscribe directly. So far, 
   the function of ouster sample drive is playing sample data set.Noy only can we 
   see the video recorded by camera,but also the point cloud frame by frame. 
 
   ```
   . ~/myworkspace/devel/setup.bash
   roslaunch ouster_ros os1.launch replay:=true viz:=true image:=true lidar_mode:=2048x10
   rosbag play *.bag --clock
   ```

- [gtsam](https://github.com/borglab/gtsam/releases) (Georgia Tech Smoothing and Mapping library, 4.0.3)
  ```
  wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.3.zip
  cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
  cd ~/Downloads/gtsam-4.0.3/
  mkdir build && cd build
  cmake -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON ..
  sudo make install
  ```

## Compile

You can use the following commands to download and compile the package.This package refer the code from git clone https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git

  ```
  cd ~/catkin_ws/src
  git clone https://github.com/haotiangu/LeGO_LOAM_OUSTER_64.git
  cd ..
  catkin_make -j1
  ```
When you compile the code for the first time, you need to add "-j1" behind "catkin_make" for generating some message types. "-j1" is not needed for future compiling.

## The system

LeGO-LOAM is speficifally optimized for a horizontally placed Ouster_64 beams on a ground vehicle. It assumes there is always a ground plane in the scan. You can replace the VLP-16 in this picture with ouster_64.

<p align='center'>
    <img src="/LeGO-LOAM/launch/jackal-label.jpg" alt="drawing" width="400"/>
</p>

This is how LeGO-LOAM works with subscribing ouster sample data set.
 
<p align='center'>
    <img src="/LeGO-LOAM/launch/information_flow.png" alt="drawing" width="400"/>
</p>


**New**: a new **useCloudRing** flag has been added to help with point cloud projection (i.e., VLP-32C, VLS-128). Velodyne point cloud has "ring" channel that directly gives the point row id in a range image. Other lidars may have a same type of channel, i.e., "r" in Ouster. If you are using a non-Velodyne lidar but it has a similar "ring" channel, you can change the PointXYZIR definition in utility.h and the corresponding code in imageProjection.cpp.

If you are using your lidar with an IMU, make sure your IMU is aligned properly with the lidar. The algorithm uses IMU data to correct the point cloud distortion that is cause by sensor motion. If the IMU is not aligned properly, the usage of IMU data will deteriorate the result. Ouster lidar IMU is not supported in the package as LeGO-LOAM needs a 9-DOF IMU. We can use the third package(IMU_TOOL) to get quaternion. There is a height gap after I subscribed the quaternion output from IMU_TOOL. The futher research will focus on sensor fusion to improve accuracy of localization.

## Run the package

1. Run the launch file:
```
roslaunch lego_loam run.launch
```
Notes: The parameter "/use_sim_time" is set to "true" for simulation, "false" to real robot usage.

2. Play existing bag files:
```
rosbag play *.bag --clock
```

## Ouster data-set

This dataset, [San Francisco data-set], is captured using a Ouster-64, which is mounted on car, on San Francisco street. The Ouster-64 rotation rate is set to 20Hz. This data-set features over 20K scans and no loop-closures. 

The picture below is the final map.
<p align='center'>
    <img src="/LeGO-LOAM/launch/map.png" alt="drawing" width="400"/>
</p>

## Cite *LeGO-LOAM*

Thank you for citing [our *LeGO-LOAM* paper](./Shan_Englot_IROS_2018_Preprint.pdf) if you use any of this code: 
```
@inproceedings{legoloam2018,
  title={LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain},
  author={Shan, Tixiao and Englot, Brendan},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4758-4765},
  year={2018},
  organization={IEEE}
}
```

## Loop Closure

The loop-closure method implemented in this package is a naive ICP-based method. It often fails when the odometry drift is too large. For more advanced loop-closure methods, there is a package called [SC-LeGO-LOAM](https://github.com/irapkaist/SC-LeGO-LOAM), which features utilizing point cloud descriptor.

## Speed Optimization

An optimized version of LeGO-LOAM can be found [here](https://github.com/facontidavide/LeGO-LOAM/tree/speed_optimization). All credits go to @facontidavide. Improvements in this directory include but not limited to:

    + To improve the quality of the code, making it more readable, consistent and easier to understand and modify.
    + To remove hard-coded values and use proper configuration files to describe the hardware.
    + To improve performance, in terms of amount of CPU used to calculate the same result.
    + To convert a multi-process application into a single-process / multi-threading one; this makes the algorithm more deterministic and slightly faster.
    + To make it easier and faster to work with rosbags: processing a rosbag should be done at maximum speed allowed by the CPU and in a deterministic way.
    + As a consequence of the previous point, creating unit and regression tests will be easier.
