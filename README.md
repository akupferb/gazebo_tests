# ROS package "gazebo_tests"

# Overview
This repo contains a package for for a roomba-like turtlebot algortihm. The robot moves forward until an object is in its path, which it then turns right until the path is clear and moves forward again. The sensor data for obstacles uses the laser scanner on the turtlebot to check for a collision and prevent it.

## Assumptions/Dependencies
The code assumes you are using *Ros Kinetic*.

The package builds using *catkin*.

The following programs/packages are used and must be installed:
* Gazebo
* Turtlebot_Gazebo package

This package relies on the following dependencies that must be installed:
* roscpp
* geometry_msgs
* std_msgs
* sensor_msgs

## Download Package
```
cd <your catkin workspace>/src
git clone https://github.com/akupferb/gazebo_tests.git
```

## Build
```
cd <your catkin workspace>
catkin_make
source devel/setup.bash
```
## Run (using roslaunch)
To run the launch file with no arguments, (this will use the default setting):
```
roslaunch gazebo_tests turtleba.launch
```

## Rosbag recording
There is a commmand-line argument available with the launch file, which you can use to enable 30 seconds of rosbag recording:

* use_rosbag (Enables="True"/Disables="False" Rosbag recording)<br/>
The default values is: *use_rosbag*="False"<br/>
**Note: *use_rosbag* only takes boolean inputs of True or False**

To modify the corresponding parameters, you use the := operator<br/>
Enabling rosbag example:
```
roslaunch gazebo_tests turtleba.launch use_rosbag:=True
```
