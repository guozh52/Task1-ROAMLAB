# Task1-ROAMLAB
The repository contains a ROS2 framework for a simple implementation of a publisher and subscriber. The publisher publishes an 8-channel EMG signal, and the subscriber receives the 20 most recent signals from this topic, calculating the average value for each channel.

## Example

![](output.gif)
## How To
OS: Ubuntu 22.04

Install ``ros-iron-desktop``. See https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html


First, move to the workspace
```
cd Task1-ROAMLAB/P1_ROAM_ws
```
In the workspace, build the package
```
colcon build
```
Source the setup file
```
source install/setup.bash
```
Run the talker node
```
ros2 run message_pub_sub talker
```
Open a new terminal, and then move to the same workspace, and source the setup file
```
cd Task1-ROAMLAB/P1_ROAM_ws
. install/setup.bash
```
Run the listener node
```
ros2 run message_pub_sub listener
```
