# RGB-D-Map-Segmentation-and-Pose-Estimation
Map point cloud segmentation using RGB-D sensor. Mapping using RTAB and doing segmentation at the same time.
This project work using yolov5 as an object detetor, using filtering to update extracted point cloud in the map frame and estimate a position to the object.
The code was designed to work out of the box with the Spot Robot body camera.

- Tested under Ubuntu 18.04 and ROS Melodic
- Require Python2.7 and Python3.8

## Getting Started

### 1.1. Setup the workspace

clone the repository to your ros workspace

```shell script
cd ~/ros_ws/src/
git clone https://github.com/sugnite/RGB-D-Map-Segmentation-and-Pose-Estimation
```

## 1.2. Setup Yolov5
Download the yolov5 for ROS package in your workspace directory.
Follow the [`instructions`](https://github.com/ch-sa/labelCloud) to setup the package.

```shell script
git clone https://github.com/sugnite/yolov5_ros
```

Download the weight of the project (require gdown).

```shell script
pip install down
cd ~/ros_ws/src/yolov5_ros/src/yolov5
gdown https://drive.google.com/uc?id=1YIUekSGq6SvQ7KzKNgbcB-D4Yt8-SDLP
```
### 1.3. Setup RAB-Map

Install rtab-map package for the mapping process

```shell script
sudo apt install ros-melodic-rtabmap-ros
```
### 1.4. Install Spot Package

Install Spot packages from clearpath

```shell script
pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core
git clone https://github.com/clearpathrobotics/spot_ros
```
### 1.5. Make your workspace

Make your workspace executable

```shell script
cd ~/ros_ws/
catkin_make
source devel/setup.bash
```
### 1.5. Download the rosbag for tests

Download the testing rosbag 

```shell script
cd ~/ros_ws/src/RGB-D-Map-Segmentation-and-Pose-Estimation/rgbd_map_segmentation_and_pose_estimation/bags
gdown https://drive.google.com/uc?id=
```

## Run the program

The program works with 2 nodes
