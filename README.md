# Welcome to AirSim

AirSim is a simulator for quadrotors (and soon other vehicles) built on Unreal Engine. Its open source, cross platform and supports hardware-in-loop with popular platforms such as PixHawk for physically and visually realistic simulations. It is developed as an Unreal plugin that can simply be dropped in to any Unreal environment you want. 

Our goal is to develop AirSim as a platform for AI research where we can experiment with deep learning, computer vision and reinforcement learning algorithms for autonomous vehicles. For this purpose, AirSim also exposes APIs to retrieve data and control vehicles in a platform independent way.

[![AirSim Demo Video](docs/images/demo_video.png)](https://youtu.be/GB-sBpXvM3s)

# Development Status

This project is under heavy development. While we are working through our backlog of new features and known issues, we welcome contributions! Our current release is in beta and our APIs are subject to change.

# How to Get It
## Prerequisites
To get the best experience you will need PixHawk or compatible device and a RC controller. These enables the "hardware-in-loop simulation" that provides more realistic experience. [Follow these instructions](docs/prereq.md) on how to get it, set it up and other alternatives.

## Windows
There are two ways to get AirSim working on your machine. Click on below links and follow the instructions.

1.  [Build it and use it with Unreal](docs/build.md)
2.  [Use the precompiled binaries](docs/use_precompiled.md)

## Linux
The official Linux build is expected to arrive in about a couple of weeks. All our current code is cross-platform and CMake enabled so please feel free to play around on other operating systems and [report any issues](issues/). We would love to make AirSim available on other platforms as well.

# How to Use It

## Manual flights
Just plugin your PixHawk (or compatible device) in your USB port, turn on the RC and press the Play button in Unreal. You should be able to control the quadrotor in the simulator with the RC and fly around. Press F1 key to view several available keyboard shortcuts.

[More detailed instructions and troubleshooting](docs/manual_flight.md)

## Gathering training data
There are two ways you can generate training data from AirSim. The easiest way is to simply press the record button on the lower right corner. This will start writing pose and images for each frame. If you would like more data logging capabilities and other features, [file a feature request](issues/) or contribute changes. The code for data logging is pretty simple to modify to your heart's desire.

More complex way to generate training data is by writing client code that uses our APIs. This allows you to be in full control of how, what, where and when you want to log data. See next section for more details.

## Programmatic control
The AirSim exposes easy to use APIs to retrive data from the quadrotor that includes ground truth, sensor data as well as various images. It also exposes APIs to control the drone in platform independent way. This allows you to use your code to control different quadrotor platforms, for example, PixHawk or DJI Matrice, without any changes. 

These APIs are also available as a part of a separate independent cross-platform library so you can deploy them on an offboard computer on your vehicle. This way you can write and test your code in simulator and later execute it on the real quadrotor. Transfer learning and related research is one of our focus areas.

[More detailed instructions for APIs](docs/apis.md)

# Contribute
We welcome contributions to help advance research frontiers. 

- [More on our design](docs/design.md)
- [More on our code structure](docs/code structure.md)

# License
This project is released under MIT License. Please review [License file](LICENSE) for more details.
