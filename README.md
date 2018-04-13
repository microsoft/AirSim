# Welcome to AirSim

AirSim is a simulator built using the Unreal Engine for drones, cars and more. It is open-source, cross-platform and supports hardware-in-loop with popular flight controllers such as PX4 for physically and visually realistic simulations. AirSim is an Unreal plugin that can be dropped into any Unreal environment.

AirSim is intended to be used as a platform for AI research and experimentation with deep learning, computer vision and reinforcement learning algorithms for autonomous vehicles. As such, AirSim provides APIs for retrieving data and controlling vehicles in a platform independent way.

**Check out the quick 1.5 minute demo**

Drones in AirSim

[![AirSim Drone Demo Video](docs/images/demo_video.png)](https://youtu.be/-WfTr1-OBGQ)

Cars in AirSim

[![AirSim Car Demo Video](docs/images/car_demo_video.png)](https://youtu.be/gnz1X3UNM5Y)

## How to Get It

### Windows
* [Download binaries](docs/use_precompiled.md)
* [Build it](docs/build_windows.md)

### Linux
* [Build it](docs/build_linux.md)

## How to Use It

### Choosing Your Vehicle: Car or Multirotor
By default, AirSim spawns a multirotor. You can easily change this to car and use all of AirSim's goodies. Please see the guide for [using a car](docs/using_car.md).

### Manual drive

If you have a remote control (RC) as shown below, you can manually control the drone in the simulator. For cars, the arrow keys can be used for manual control.

[More details](docs/remote_control.md)

![record screenshot](docs/images/AirSimDroneManual.gif)

![record screenshot](docs/images/AirSimCarManual.gif)


### Programmatic control

AirSim provides APIs that allow programmatic interaction with vehicles in the simulation. These APIs can be used to retrieve images and states, control vehicles, etc. The APIs are exposed through RPC, and are accessible using languages including C++, Python, C# and Java.

These APIs are also available as a part of a separate, independent cross-platform library, and can be deployed on an companion computer in your vehicle. This allows code to be written and tested in a simulator before it is executed on real vehicles. Transfer learning and related research is one of our focuses.

[More details](docs/apis.md)

### Gathering training data

There are two ways you can generate deep learning training data from AirSim. The easiest way is to press the record button in the lower right corner, which will start the writing of poses and images from each frame. The data logging code is simple and can be modified as required.

![record screenshot](docs/images/record_data.png)

A better way to generate training data is through the APIs, which provide full control of how, what, where and when data is logged. 

### Computer Vision mode

AirSim's "Computer Vision" mode does not have vehicle physics or dynamics, but the keyboard can be used to move around. The APIs can be used to position a vehicle in an arbitrary pose, and capture images such as depth, disparity, surface normals or object segmentation. 

[More details](docs/image_apis.md)

### Tutorials

- [Video - Setting up AirSim with Pixhawk Tutorial](https://youtu.be/1oY8Qu5maQQ) by Chris Lovett
- [Video - Using AirSim with Pixhawk Tutorial](https://youtu.be/HNWdYrtw3f0) by Chris Lovett
- [Video - Using off-the-self environments with AirSim](https://www.youtube.com/watch?v=y09VbdQWvQY) by Jim Piavis
- [Reinforcement Learning with AirSim](docs/reinforcement_learning.md) by Ashish Kapoor
- [The Autonomous Driving Cookbook](https://aka.ms/AutonomousDrivingCookbook) by Microsoft Deep Learning and Robotics Garage Chapter
- [Using TensorFlow for simple collision avoidance](https://github.com/simondlevy/AirSimTensorFlow) by Simon Levy and WLU team


## What's New

* We now have the [car model](docs/using_car.md).
* No need to build the code. Just download [binaries](https://github.com/Microsoft/AirSim/releases) and you are good to go!
* The [reinforcement learning example](docs/reinforcement_learning.md) with AirSim
* New built-in flight controller called [simple_flight](docs/simple_flight.md) that "just works" without any additional setup. It is also now *default*. 
* AirSim now also generates [depth as well as disparity images](docs/image_apis.md) that is in camera plan. 
* We also have official Linux build now! If you have been using AirSim with PX4, you might want to read the [release notes](docs/release_notes.md).

## Participate

### Paper

Additional technical details on AirSim are available in the [AirSim paper (FSR 2017 Conference)](https://arxiv.org/abs/1705.05065). Please cite this as:
```
@inproceedings{airsim2017fsr,
  author = {Shital Shah and Debadeepta Dey and Chris Lovett and Ashish Kapoor},
  title = {AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles},
  year = {2017},
  booktitle = {Field and Service Robotics},
  eprint = {arXiv:1705.05065},
  url = {https://arxiv.org/abs/1705.05065}
}
```

### Contribute

If you would like to contribute to AirSim, please look at the [open issues](https://github.com/microsoft/airsim/issues) and [Trello board](https://trello.com/b/1t2qCeaA/todo).

* [More on AirSim design](docs/design.md)
* [More on code structure](docs/code_structure.md)
* [Contribution Guidelines](docs/contributing.md)


### Who is Using AirSim?

We [maintain a list](docs/who_is_using.md) of the projects, people and groups using AirSim that we are aware of. If you would like to be featured in this list, please [make a request here](https://github.com/microsoft/airsim/issues).

## Contact

Join the [AirSim group at Facebook](https://www.facebook.com/groups/1225832467530667/) to stay up-to-date or ask any questions.

## FAQ

If you run into problems, check the [FAQ](docs/faq.md), or feel free to post issues on the [AirSim github](https://github.com/Microsoft/AirSim/issues).

## License

This project is released under MIT License. Please review [License file](LICENSE) for more details.
