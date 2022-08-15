# RGB-D-Map-Segmentation-and-Pose-Estimation
Map point cloud segmentation using RGB-D sensor. 
The mapping is using RTAB and do the segmentation at the same time.
This project work using yolov5 as an object detetor, using filtering to update extracted point cloud in the map frame and estimate a position to the object.
The code was designed to work out of the box with the Spot Robot body camera.

- Tested under Ubuntu 18.04 and ROS Melodic
- Require Python2.7 and Python3.8

![output](./docs/output_package.png)

## Getting Started

### 1.1. Setup the workspace

clone the repository to your ros workspace

```shell script
cd ~/ros_ws/src/
git clone https://github.com/sugnite/RGB-D-Map-Segmentation-and-Pose-Estimation
```

### 1.2. Setup Yolov5
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
### 1.6. Download the rosbag for tests (optional)

Download the testing rosbag 

```shell script
cd ~/ros_ws/src/RGB-D-Map-Segmentation-and-Pose-Estimation/rgbd_map_segmentation_and_pose_estimation/bags
gdown https://drive.google.com/uc?id=1tAV1GYfxH25EosJLpR4FSFkXNUuFxjuB
```

![mapping](./docs/mapping.gif)

## Run the program

The program work with 2 files to start in both python2 an python3

### 2.1. Run the Demo bag

Setup sim parameters

First Terminal
```shell script
roscore
```
Second Terminal
```shell script
rosparam set use_sim_time true
roscd rgbd_map_segmentation_and_pose_estimation/bags 
rosbag play --clock spot_demo_bag.bag
```
The next launch file will start the object detection network and the 3D filtering of the extracted point cloud

![detection](./docs/Stairs_exp1.png)

Third Terminal
```shell script
roslaunch rgbd_map_segmentation_and_pose_estimation semantic_classifier.launch
```

The last file will start the point cloud extraction and 3D pose estimation of the segmented objects

Fourth Terminal
```shell script
roslaunch rgbd_map_segmentation_and_pose_estimation second_node_semantic_classifier.launch
```

