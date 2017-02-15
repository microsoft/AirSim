# Welcome to AirSim

AirSim is a simulator for drones (and soon other vehicles) built on Unreal Engine. Its open source, 
cross platform and supports hardware-in-loop with popular platforms such as PixHawk for physically 
and visually realistic simulations. It is developed as Unreal plugin that can be just dropped in to any 
Unreal environment you want. It also exposes APIs so you can retrieve sensor data, ground truth and camera 
images from the simulator as well as send control commands to the vehicle. Our goal is to develop AirSim as
a platform for AI research where we can experiment with deep learning and reinforcement learning algorithms
for autonomous vehicles.

# Development Status

This project is under active development. While we are working through our backlog of new features and known issues, 
we welcome contributions! Our current release is in beta and our APIs are subject to change.

# How to Get It
## Prerequisites
Currently, to get the best experience you will need PixHawk or compatible PX4 based device and a RC controller. 
These enable so called "hardware-in-loop (HITL) simulation"  that provides more realistic experience. 
[Follow these instructions](docs/hil_setup.md) on how to get it, set it up and other alternatives.

## Windows
There are two ways to get AirSim working on your machine.

1.  [Use the precompiled binaries](docs/use_precompiled.md)
2.  [Build it yourself](docs/build.md)

We recommend option 2 if you are want to use the Unreal plugin in your own environment.

#Linux
All our current code is cross-platform and CMake enabled. We are working on to iron out few rough edges for our
official Linux build and expect to release it around next two weeks. Meanwhile, please feel free to play around 
on other operating systems and report any issues. We had love to make AirSim available on as many platforms as possible.

# How to Use It

## Manual flights

## Gathering training data

## Programmatic control

# Contribute

# Acknowledgements

# Licence

