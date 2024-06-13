# MOIRO AGV
## 0. Intro
- ROS2 Humble Packages for MOIRO AGV
- 해당 프로젝트는 Moiro에서 사용된 자체제작 AGV를 ROS 2 Humble 환경에서 제어하기 위한 것입니다.

## 1. Installation
### 1) Source Build
```sh
cd moiro_ws/src
git clone https://github.com/MOIRO-KAIROS/moiro_agv.git
cd moiro_ws
colcon build
```
### 2) Dependency Package
다음 패키지들이 필요합니다.
- cartographer-ros
- navigation2
- nav2-bringup
```sh
sudo apt-get install -y ros-humble-cartographer-ros ros-humble-navigation2 ros-humble-nav2-bringup
```
## 2. Usage
### 1) Teleoperation

```sh
ros2 launch moiro_agv_bringup moiro_agv.launch.py
ros2 run moiro_agv_teleop teleop_keyboard 
```
### 2) SLAM

```sh
ros2 launch moiro_agv_cartographer cartographer.launch.py
```
