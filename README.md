# AllEye
EECE5552 Assistive Robotics Final Project

## Installation

``` bash
sudo apt install ros-noetic-apriltag-ros
```

## Building

``` bash
catkin build
```

## Running

Run our nodes and start the drone. Remember to always call `source devel/setup.bash`

``` bash
source devel/setup.bash
roslaunch sjtu_drone simple.launch
```

``` bash
source devel/setup.bash
roslaunch alleye start_drone.launch
```
