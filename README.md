# Welcome to AirSim

AirSim is a simulator for drones (and soon other vehicles) built on Unreal Engine. It is open-source, cross platform and supports hardware-in-loop with popular flight controllers such as Pixhawk for physically and visually realistic simulations. It is developed as an Unreal plugin that can simply be dropped in to any Unreal environment you want. 

Our goal is to develop AirSim as a platform for AI research to experiment with deep learning, computer vision and reinforcement learning algorithms for autonomous vehicles. For this purpose, AirSim also exposes APIs to retrieve data and control vehicles in a platform independent way.

**Check out the quick 1.5 minute demo**

[![AirSim Demo Video](docs/images/demo_video.png)](https://youtu.be/-WfTr1-OBGQ)

# Development Status

This project is under heavy development. While we are working through our backlog of new features and known issues, we welcome contributions! Our current release is in beta and our APIs are subject to change.

# How to Get It
## Prerequisites
To get the best experience you will need Pixhawk or compatible device and a RC controller. This enables the "hardware-in-loop simulation" for more realistic experience. [Follow these instructions](docs/prereq.md) on how to get it, set it up and other alternatives.

## Windows
There are two ways to get AirSim working on your machine. Click on below links and follow the instructions.

1.  [Build it and use it with Unreal](docs/build.md)
2.  [Use the precompiled binaries](docs/use_precompiled.md)

## Linux
The official Linux build is expected to arrive in about a couple of weeks. All our current code is cross-platform and CMake enabled so please feel free to play around on other operating systems and [report any issues](https://github.com/Microsoft/AirSim/issues). We would love to make AirSim available on other platforms as well.

# How to Use It

## Manual flights
Follow the steps above to build and set up the Unreal environment. Plugin your Pixhawk (or compatible device) in your USB port, turn on the RC and press the Play button in Unreal. You should be able to control the drones in the simulator with the RC and fly around. Press F1 key to view several available keyboard shortcuts.

[More details](docs/manual_flight.md)

## Gathering training data
There are two ways you can generate training data from AirSim for deep learning. The easiest way is to simply press the record button on the lower right corner. This will start writing pose and images for each frame. 
![record screenshot](docs/images/record_data.png)
If you would like more data logging capabilities and other features, [file a feature request](https://github.com/Microsoft/AirSim/issues) or contribute changes. The data logging code is pretty simple and you can modify it to your heart's desire.

A more complex way to generate training data is by writing client code that uses our APIs. This allows you to be in full control of how, what, where and when you want to log data. See the next section for more details.

## Programmatic control
The AirSim exposes easy to use APIs in order to retrive data from the drones that includes ground truth, sensor data as well as various images. It also exposes APIs to control the drones in a platform independent way. This allows you to use your code to control different drones platforms, for example, Pixhawk or DJI Matrice, without making changes as well as without having to learn internal protocols details. 

These APIs are also available as a part of a separate independent cross-platform library so you can deploy them on an offboard computer on your vehicle. This way you can write and test your code in simulator and later execute it on the real drones. Transfer learning and related research is one of our focus areas.

[More details](docs/apis.md)

# Paper
You can get additional technical details in [our paper (work in progress)](https://www.microsoft.com/en-us/research/wp-content/uploads/2017/02/aerial-informatics-robotics-TR.pdf). Please cite this as:
```
@techreport{MSR-TR-2017-9,
     title = {{A}erial {I}nformatics and {R}obotics Platform},
     author = {Shital Shah and Debadeepta Dey and Chris Lovett and Ashish Kapoor},
     year = {2017},
     institution = {Microsoft Research},
     number = {{M}{S}{R}-{T}{R}-2017-9}}
}
```

# Contribute
We welcome contributions to help advance research frontiers. 

- [More on our design](docs/design.md)
- [More on our code structure](docs/code_structure.md)

# License
This project is released under MIT License. Please review [License file](LICENSE) for more details.
