# AllEye
EECE5552 Assistive Robotics Final Project

## Installation
[AirSim](https://microsoft.github.io/AirSim/build_linux/)

``` bash
sudo apt install git-lfs ros-noetic-apriltag ros-noetic-apriltag-ros
```

## Building

``` bash
source /path/to/AirSim/ros/devel/setup.bash
cd final_ws
catkin build
```

## Running

``` bash
roslaunch airsim_ros_pkgs airsim_node.launch
```
