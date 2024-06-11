# moiro_agv
## Intro
ROS2 Humble Packages for MOIRO AGV
## Installation
### Source Build
```sh
cd moiro_ws/src
git clone https://github.com/MOIRO-KAIROS/moiro_agv.git
cd moiro_ws
colcon build
```
### Dependency Package
다음 패키지들이 필요합니다.
- cartographer-ros
- navigation2
- nav2-bringup
```sh
sudo apt-get install -y ros-humble-cartographer-ros ros-humble-navigation2 ros-humble-nav2-bringup
```
## Result
### Teleoperation

```sh
ros2 launch moiro_agv_bringup moiro_agv.launch.py
ros2 run moiro_agv_teleop teleop_keyboard 
```
### SLAM

```sh
ros2 launch moiro_agv_cartographer cartographer.launch.py
```
