# AllEye
EECE5552 Assistive Robotics Final Project

## Installation
[AirSim](https://microsoft.github.io/AirSim/build_linux/)

``` bash
sudo apt install git-lfs ros-noetic-apriltag ros-noetic-apriltag-ros
```

## Building

**NOTE: Replace /path/to/AirSim with your path**

``` bash
source /path/to/AirSim/ros/devel/setup.bash
cd final_ws
catkin build
```

## Running

Start Unreal Engine and open Blocks.uproject in Environments/Blocks.

Copy our settings.json to the Documents/AirSim folder

``` bash
cp drone.json ~/Documents/AirSim/settings.json
```

Run our nodes and start the drone. Remember to always call `source devel/setup.bash`

``` bash
roslaunch alleye start_drone.launch
```
