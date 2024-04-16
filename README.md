# ORB_SLAM3_ROS2

## Requirements
  - Ubuntu 22.04
  - ROS2 Humble
  - OpenCV 4.5.4

## How to build
1. Building ORB-SLAM3 library
```
cd ~/
git clone https://github.com/vibeus/ORB_SLAM3.git -b ubuntu_22_04

cd ~/ORB_SLAM3
./build.sh
```
2. Build ORB_SLAM3 ROS2
```
mkdir -p ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/vibeus/ORB_SLAM3.git -b ros2_humble

cd ~/ros2_ws
colcon build --symlink-install --packages-select orbslam3
```

## Troubleshootings
1. Jetson orin nx `OpenCV 4.8`: try not pre-install `Jetson SDK Components`
