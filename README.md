# YOLOv5 ROS
This is a ROS interface for using YOLOv5 for real time object detection on a ROS image topic. It supports inference on multiple deep learning frameworks used in the [official YOLOv5 repository](https://github.com/ultralytics/yolov5).

This Repository is forked from [mats-robotics/yolov5_ros](https://github.com/mats-robotics/yolov5_ros).

## Installation

### ROS Installation
**Method 1（Recommend）**: `quick install`
```bash
# 鱼香 ROS
wget http://fishros.com/install -O fishros && . fishros
```
**Method 2**：`step by step install`
```bash
# step 1: 添加ros软件源
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# step 2: 添加秘钥
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# step 3：添加完源后记得更新一下
sudo apt update
# step 4: 安装 ros
# 查看自己的 ubuntu 版本,安装与自己ubuntu版本匹配的 ROS 版本
lsb_release -a
# ubuntu 20.04 安装 noetic「完整桌面版安装」
sudo apt install ros-noetic-desktop-full
```

### Dependencies
This package is built and tested on Ubuntu 20.04 LTS and ROS Noetic with Python 3.8.

* Clone the packages to ROS workspace and install requirement for YOLOv5 submodule:

**Method 1**: follow [official YOLOv5 repository](https://github.com/ultralytics/yolov5)
```bash
cd <ros_workspace>/src
git clone https://github.com/mats-robotics/detection_msgs.git
git clone --recurse-submodules https://github.com/mats-robotics/yolov5_ros.git 
cd yolov5_ros/src/yolov5
pip install -r requirements.txt # install the requirements for yolov5
```

**Method 2** : `Recommend` 
```bash
git clone https://github.com/chelixuan/yolov5_ros_ws.git
cd yolov5_ros_ws/src/yolov5
pip install -r requirements.txt 
pip install empy==3.3.2
pip install catkin-tools rospkg
```

* Build the ROS package:
**conda envs**
```bash
cd yolov5_ros_ws
which python # 查看当前 python 的地址，例如得到 /home/chelx/.conda/envs/ros/bin/python
catkin_make -DPYTHON_EXECUTABLE=/home/chelx/.conda/envs/ros/bin/python # 使用当前环境的python，编译
# 编译过程中，在工作空间的根目录里会自动产生build和devel两个文件夹及其中的文件
# 使用source命令运行devel中的 setup.bash 脚本文件，使工作空间中的环境变量可以生效
source devel/setup.bash
# 为了确保环境变量已经生效，可以使用如下命令进行检查：
echo $ROS_PACKAGE_PATH
```

* Make the Python script executable 
```bash
cd <ros_workspace>/src/yolov5_ros/src
chmod +x detect.py
```

## Basic usage
Change the parameter for `input_image_topic` in launch/yolov5.launch to any ROS topic with message type of `sensor_msgs/Image` or `sensor_msgs/CompressedImage`. Other parameters can be modified or used as is.

* Launch the node:
```bash
rosbag play <your-rosbag.bag>
roslaunch yolov5_ros yolov5.launch
```

* visualize outputs:
```bash
rosnode list
rosnode info /detect
rviz

rostopic echo /yolov5/detections
```

## Using custom weights and dataset (Working)
* Put your weights into `yolov5_ros/src/yolov5`
* Put the yaml file for your dataset classes into `yolov5_ros/src/yolov5/data`
* Change related ROS parameters in yolov5.launch: `weights`,  `data`

## Reference
* YOLOv5 official repository: https://github.com/ultralytics/yolov5
* YOLOv3 ROS PyTorch: https://github.com/eriklindernoren/PyTorch-YOLOv3
* Darknet ROS: https://github.com/leggedrobotics/darknet_ros
