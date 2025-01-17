# Deep Grasp Demo
# Made a couple changes such that this actually builds -J

<img src="https://picknik.ai/assets/images/logo.jpg" width="120">



1) [Overview](#Overview) </br>
2) [Packages](#Packages) </br>
3) [Getting Started](#Getting-Started) </br>
4) [Install Grasp Pose Detection](#Install-Grasp-Pose-Detection) </br>
5) [Install Dex-Net](#Install-Dex-Net) </br>
6) [Download ROS Packages](#Download-ROS-Packages) </br>
7) [Launching Demos and Further Details](#Launching-Demos-and-Further-Details) </br>
8) [Depth Sensor Data](#Depth-Sensor-Data) </br>
9) [Camera View Point](#Camera-View-Point) </br>
10) [Known Issues](#Known-Issues) </br>

## Overview
This repository contains several demos
using deep learning methods for grasp pose generation within the MoveIt Task Constructor.

The packages were developed and tested on Ubuntu 18.04 running ROS Melodic.


## Packages
* [deep_grasp_task](https://github.com/PickNikRobotics/deep_grasp_demo/tree/master/deep_grasp_task): constructs a pick and place task using deep learning methods
for the grasp generation stage within the MoveIt Task Constructor

* [moveit_task_constructor_dexnet](https://github.com/PickNikRobotics/deep_grasp_demo/tree/master/moveit_task_constructor_dexnet): uses [Dex-Net](https://berkeleyautomation.github.io/dex-net/) to sample grasps from a depth image

* [moveit_task_constructor_gpd](https://github.com/PickNikRobotics/deep_grasp_demo/tree/master/moveit_task_constructor_gpd): uses [GPD](https://github.com/atenpas/gpd) to sample grasps from 3D point clouds


## Getting Started
First, Complete the [Getting Started Tutorial](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html).
Make sure you follow the melodic portion of the tutorials
Before installing the dependencies it is recommended to run:
```
sudo apt update
sudo apt upgrade
```

**Important Note**: It is recommended to install dependencies that are not ROS packages outside of the
catkin workspace. For GPD this includes PCL, OpenCV, and the GPD library. For Dex-Net this includes [gqcnn](https://github.com/BerkeleyAutomation/gqcnn), [autolab_core](https://github.com/BerkeleyAutomation/autolab_core), [perception](https://github.com/BerkeleyAutomation/perception), and [visualization](https://github.com/BerkeleyAutomation/visualization). The steps bellow will walk you through the installation.

## Only install dexnet for now, since we'd like to use a depth image
## Install Dex-Net
1) It is recommended to upgrade pip and to create a virtual environment
   prior to running the install script in the next step.
   
   Make the venv environment inside your catkin workspace (same level as src, build, devel)
   ```
   python3 -m venv tutorial-env
   source tutorial-env/bin/activate
   ```
   
   ```
   python3 -m pip install --upgrade pip
   ```

2) Run the install script to download the requirements </br>
  If you have a GPU this option will install tensorflow with GPU support. This script
  will install packages for Python 3.
  ```
  cd tutorial-env
  wget https://raw.githubusercontent.com/jeoseo/deep_grasp_demo/master/dexnet_install.sh
  wget https://raw.githubusercontent.com/jeoseo/deep_grasp_demo/master/dexnet_requirements.txt
  chmod +x dexnet_install.sh
  ./dexnet_install.sh {cpu|gpu}
  ```
  
  Most likely, you will just be using the cpu, so remove the brackets and |gpu from the last command.

3) Download the pretrained models

  ```
  cd ..
  ./src/gqcnn_dependencies/gqcnn/scripts/downloads/models/download_models.sh
  ```

4) Configuration File Paths
## simply replace bostoncreek with your linux username, in the absolute paths

In the file 'dexnet_config.yaml', change the absolute path from /home/jeoseo/ws_grasp... to /home/your_username/your_ws_name...
This is because they decided to use an absolute path :/

### TODO, fix any tutorials below

### Panda Gazebo Support (Optional)
You will need the C++ Franka Emika library. This can be installed from [source](https://github.com/frankaemika/libfranka) or by executing:
```
sudo apt install ros-melodic-libfranka
```

You will need two additional packages.
```
git clone https://github.com/tahsinkose/panda_moveit_config.git -b melodic-devel
git clone https://github.com/tahsinkose/franka_ros.git -b simulation
```


## Launching Demos and Further Details
To see how to launch the demos using GPD and Dex-Net see the [moveit_task_constructor_gpd](https://github.com/jeoseo/deep_grasp_demo/tree/master/moveit_task_constructor_gpd) and [moveit_task_constructor_dexnet](https://github.com/jeoseo/deep_grasp_demo/tree/master/moveit_task_constructor_dexnet) packages.


## Depth Sensor Data
### Collecting Data using Gazebo
Perhaps you want to collect depth sensor data on an object and use fake controllers to execute the motion plan. The launch file `sensor_data_gazebo.launch` will launch a `process_image_server` and a `point_cloud_server` node. These will provide services to save either images or point clouds.
Images will be saved to `moveit_task_constructor_dexnet/data/images` and point clouds saved to `moveit_task_constructor_gpd/data/pointclouds`.

To collect either images or point clouds run:
```
roslaunch deep_grasp_task sensor_data_gazebo.launch
```

To save the depth and color images:
```
rosservice call /save_images "depth_file: 'my_depth_image.png'
color_file: 'my_color_image.png'"

```

To save a point cloud:
```
rosservice call /save_point_cloud "cloud_file: 'my_cloud_file.pcd'"
```


## Camera View Point
Initially, the camera is setup to view the cylinder from the side of the robot. It is useful particularly for Dex-Net to place the camera in an overhead position above the object. To change the camera view point there are a few files to modify. You can move the camera to a preset overhead position or follow the general format to create a new position.

First, modify the camera or the panda + camera urdf.

If you want to move the camera position just for collecting sensor data, in `deep_grasp_task/urdf/camera/camera.urdf.xacro` change the camera xacro macro line to read:
```XML
<xacro:kinect_camera parent_link="world" cam_px="0.5" cam_pz="0.7" cam_op="1.57079632679"/>
```

If you want to move the camera position and use the robot to execute trajectories. Go to `deep_grasp_task/urdf/robots/panda_camera.urdf.xacro` and change the camera xacro macro line to read:
```XML
<xacro:kinect_camera parent_link="panda_link0" cam_px="0.5" cam_pz="0.7" cam_op="1.57079632679"/>
```

Next, specify the transformation from the robot base link to the camera link.

Change `deep_grasp_task/config/calib/camera.yaml` to read:
```YAML
trans_base_cam: [0.500, 0.000, 0.700, 0.707, 0.000, 0.707, 0.000]
```

Finally, this is optional depending on whether the camera is added to the planning scene. If the camera is in the planning scene you need to modify `deep_grasp_task/config/panda_object.yaml` to read:
```YAML
spawn_camera: true
camera_pose: [0.5, 0, 0.7, 0, 1.571, 1.571]
```

## Known Issues
1) When running with Gazebo
```
ros.moveit_simple_controller_manager.SimpleControllerManager: Controller panda_hand_controller failed with error GOAL_TOLERANCE_VIOLATED:
ros.moveit_ros_planning.trajectory_execution_manager: Controller handle panda_hand_controller reports status ABORTED
```

2) Planning may fail

If using GPD, increase the number of points sampled by setting `num_samples` in `config/gpd_config.yaml`.
Another option is to run either algorithm again. Maybe low quality grasps were sampled or they were not kinematically feasible.
