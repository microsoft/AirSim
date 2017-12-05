# Welcome to AirSim

AirSim is a simulator for drones, cars and more built on Unreal Engine. It is open-source, cross platform and supports hardware-in-loop with popular flight controllers such as PX4 for physically and visually realistic simulations. It is developed as an Unreal plugin that can simply be dropped in to any Unreal environment you want.

Our goal is to develop AirSim as a platform for AI research to experiment with deep learning, computer vision and reinforcement learning algorithms for autonomous vehicles. For this purpose, AirSim also exposes APIs to retrieve data and control vehicles in a platform independent way.

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

### Chosing Your Vehicle: Car or Multirotor
By default AirSim spawns multirotor. You can easily change this to car and use all of AirSim goodies. Please see [using car](docs/using_car.md) guide.

### Manual drive

If you have remote control (RC) as shown below, you can manually control the drone in the simulator. For cars, you can use arrow keys to drive manually.

[More details](docs/remote_control.md)

![record screenshot](docs/images/AirSimDroneManual.gif)

![record screenshot](docs/images/AirSimCarManual.gif)


### Programmatic control

AirSim exposes APIs so you can interact with vehicle in the simulation programmatically. You can use these APIs to retrieve images, get state, control the vehicle and so on. The APIs are exposed through RPC and accessible via variety of languages including C++, Python, C# and Java.

These APIs are also available as a part of a separate independent cross-platform library so you can deploy them on an companion computer on your vehicle. This way you can write and test your code in simulator and later execute it on the real vehicles. Transfer learning and related research is one of our focus areas.

[More details](docs/apis.md)

### Gathering training data

There are two ways you can generate training data from AirSim for deep learning. The easiest way is to simply press the record button on the lower right corner. This will start writing pose and images for each frame. The data logging code is pretty simple and you can modify it to your heart's desire.

![record screenshot](docs/images/record_data.png)

A better way to generate training data exactly the way you want is by accessing the APIs. This allows you to be in full control of how, what, where and when you want to log data. 

### Computer Vision mode

Yet another way to use AirSim is so-called "Computer Vision" mode. In this mode, you don't have vehicle physics and dynamics but you can use keyboard to move around and use APIs to position the vehicle in any arbitrary pose and get images such as depth, disparity, surface normals or object segmentation. 

[More details](docs/image_apis.md)

### Tutorials

- [Video - Setting up AirSim with Pixhawk Tutorial](https://youtu.be/1oY8Qu5maQQ) 
- [Video - Using AirSim with Pixhawk Tutorial](https://youtu.be/HNWdYrtw3f0)
- [Reinforcement Learning with AirSim](docs/reinforcement_learning.md)

## What's New

* We now have the [car model](docs/using_car.md).
* No need to build the code. Just download [binaries](https://github.com/Microsoft/AirSim/releases) and you are good to go!
* The [tutorial](https://www.youtube.com/watch?v=y09VbdQWvQY) for using off-the-self environments with AirSim.
* The [reinforcement learning example](docs/reinforcement_learning.md) with AirSim
* New built-in flight controller called [simple_flight](docs/simple_flight.md) that "just works" without any additional setup. It is also now *default*. 
* AirSim now also generates [depth as well as disparity images](docs/image_apis.md) that is in camera plan. 
* We also have official Linux build now! If you have been using AirSim with PX4, you might want to read the [release notes](docs/release_notes.md).

## Participate

### Paper

More technical details are available in [AirSim paper (FSR 2017 Conference)](https://arxiv.org/abs/1705.05065). Please cite this as:
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

Please take a look at [open issues](https://github.com/microsoft/airsim/issues) and [Trello board](https://trello.com/b/1t2qCeaA/todo) if you are looking for areas to contribute to.

* [More on AirSim design](docs/design.md)
* [More on code structure](docs/code_structure.md)
* [Contribution Guidelines](docs/contributing.md)


### Who is Using AirSim?

We are [maintaining list](docs/who_is_using.md) of few projects, people and groups that we are aware of. If you had like to be featured in this list please [add request here](https://github.com/microsoft/airsim/issues).

## Contact

Join the [AirSim group at Facebook](https://www.facebook.com/groups/1225832467530667/) to stay up to date or ask any questions.

## FAQ

If you run into problems, check the [FAQ](docs/faq.md) and feel free to post issues on the [AirSim github](https://github.com/Microsoft/AirSim/issues).

## License

This project is released under MIT License. Please review [License file](LICENSE) for more details.
