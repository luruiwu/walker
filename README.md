# Walker -  A simple obstacle avoidance algorithm for a turtlebot.
[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

This package implements a simple walker algorithm on a TurtleBot to emulate the behavior of a roomba vacuum cleaner robot. The robot moves forward in an environment, and when a potential collision with an obstacle is expected, it rotates in place until it is safe to navigate straight again.

## Dependencies
This is a ROS package which needs 
- [ROS Kinetic](http://wiki.ros.org/kinetic) to be installed on Ubuntu 16.04. Installation instructions are outlined [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).
- [Turtlebot](https://www.turtlebot.com/) packages are required. Run the following command to install all turtlebot related packages.
```
sudo apt-get install ros-kinetic-turtlebot*
```
- [Gazebo](http://wiki.ros.org/gazebo_ros_pkgs) version 7.0.0 or above. Installation instructions can be found [here](http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b1).

## Build
In your desired directory, please run the following commands.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/mjerrar/walker
cd ..
catkin_make
```
## Demo
To run the simulation, run the following commands.
```
cd <path to catkin_ws>
source devel/setup.bash
roslaunch walker walk.launch
```
This launches Gazebo with a standard TurtleBot world. The walker node publishes navigating commands to TurtleBot and the TurtleBot starts moving.

To terminate the simulation, CTRL+C in the launch terminal.

## ROSBAG
### Record
To record the simulation, execute the launch file with an added argument as shown below.
```
roslaunch walker walk.launch rec:=true
```
This will record a rosbag containing all the topics except the /camera/* topics and save it in the results sub directory. To terminate the simulation and/or the recording CTRL+C in the launch terminal.

A sample [rosbag](https://github.com/mjerrar/walker/blob/master/results/recorded.bag) recorded for ~30 seconds exists in the results sub-directory. 

### Inspect
To inspect the topics and messages in the rosbag, navigate to the results sub-directory and print the bag info.
```
cd <path to catkin_ws>/src/walker/results
rosbag info recorded.bag
```
### Play
To play the rosbag, start the ROS master in a terminal.
```
roscore
```
In a new terminal, navigate to the results sub-directory of the package and play the rosbag.
```
cd <path to catkin_ws>/src/walker/results
rosbag play recorded.bag
```
To terminate the rosbag play, CTRL+C in the terminal.
