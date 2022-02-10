# Deep Grasp Demo
<img src="https://picknik.ai/assets/images/logo.jpg" width="120">

This is a fork from Picknik that we have adapted for use on the XArm Permobil senior design project.

## Overview
This repository contains several demos
using deep learning methods for grasp pose generation within the MoveIt Task Constructor.

The packages were developed and tested on Ubuntu 18.04 running ROS Melodic.


## Packages
* [deep_grasp_task](https://github.com/PickNikRobotics/deep_grasp_demo/tree/master/deep_grasp_task): constructs a pick and place task using deep learning methods
for the grasp generation stage within the MoveIt Task Constructor


* [moveit_task_constructor_gpd](https://github.com/PickNikRobotics/deep_grasp_demo/tree/master/moveit_task_constructor_gpd): uses [GPD](https://github.com/atenpas/gpd) to sample grasps from 3D point clouds


## Install Grasp Pose Detection
1) Requirements

  Ideally, do these installations outside of your catkin workspace. You can make a separate folder outside of your workspace with something like...
  ```
  cd ..
  mkdir gpd_installations
  ```
  * PCL >= 1.9: The `pcl_install.sh` script will install PCL 1.11
  ```
  wget https://raw.githubusercontent.com/PickNikRobotics/deep_grasp_demo/master/pcl_install.sh
  chmod +x pcl_install.sh
  sudo ./pcl_install.sh
  ```

  * OpenCV >= 3.4: The `opencv_install.sh` script will install OpenCV 3.4
  ```
  wget https://raw.githubusercontent.com/PickNikRobotics/deep_grasp_demo/master/opencv_install.sh
  chmod +x opencv_install.sh
  sudo ./opencv_install.sh
  ```

  * Eigen >= 3.0: If ROS is installed then this requirement is satisfied


