# How to use AirSim with Robot Operating System (ROS)

AirSim and ROS can be integrated using C++ or Python.  Some example ROS nodes are provided demonstrating how to publish data from AirSim as ROS topics.

# Python

## Prerequisites

These instructions are for Ubuntu 16.04, ROS Kinetic, UE4 4.18 or higher, and latest AirSim release.
You should have these components installed and working before proceeding

## Setup


### Create a new ROS package in your catkin workspace following these instructions.  

Create a new ROS package called airsim or whatever you like.

[Create ROS package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)

If you don't already have a catkin workspace, you should first work through the ROS beginner tutorials.

### Add AirSim ROS node examples to ROS package

In the ROS package directory you made, copy the ros examples from the AirSim/PythonClient directory to your ROS package. Change the code below to match your AirSim and catkin workspace paths.

```
# copy package
mkdir -p ../catkin_ws/src/airsim/scripts/airsim
cp AirSim/PythonClient/airsim/*.py ../catkin_ws/src/airsim/scripts/airsim

# copy ROS examples
cp AirSim/PythonClient/ros/*.py ../catkin_ws/src/airsim/scripts
```


### Build ROS AirSim package

Change directory to your top level catkin workspace folder i.e. ```cd ~/catkin_ws```  and run ```catkin_make```
This will build the airsim package.  Next, run ```source devel/setup.bash``` so ROS can find the new package.
You can add this command to your ~/.bashrc to load your catkin workspace automatically.

## How to run ROS AirSim nodes

First make sure UE4 is running an airsim project, the car or drone should be selected, and the simulations is playing.
Examples support car or drone.  Make sure to have the correct vehicle for the ros example running. 

The example airsim nodes can be run using ```rosrun airsim example_name.py``` The output of the node 
can be viewed in another terminal by running ```rostopic echo /example_name```  You can view a list of the
topics currently published via tab completion after typing ```rostopic echo``` in the terminal.
Rviz is a useful visualization tool that can display the published data. 

### Troubleshooting

In the case of ```rosrun airsim example_name.py``` returning ```Couldn't find executable named...``` you may ```chmod +x example_name.py``` to tell the system that this is executable.


# C++ (coming soon)
