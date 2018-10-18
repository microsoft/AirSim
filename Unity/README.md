# Airsim Unity

Airsim Unity allows you to run your simulators in the [Unity Engine](https://unity3d.com/). This project comes with some sample Unity projects and a wrapper around the AirLib library to run as a [managed plugin](https://docs.unity3d.com/Manual/UsingDLL.html) in Unity.

## Getting Started

### Pick a Unity Project
This repository comes with two very basic Unity Projects, for a Car simulator and a Drone simulator. They are meant to be very lightweight, and can be used to verify your setup is correct.

Once you have things working in the basic projects, you can move on to the more detailed Unity projects. They can be found in the [Releases](https://github.com/Microsoft/AirSim/releases) tab. 

### Run the AirLib setup
Follow the instructions found [here](https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md). You can skip the Unreal steps.

### Windows

#### Download it
* Pick a a project of your choice. 
* Download the `AirLibWrapper.dll` at the [Releases](https://github.com/Microsoft/AirSim/releases) page.
* If you are running a Drone project, download the `drone.obj` from the Releases page and place at path `Unity\DroneDemo\Assets\AirSimAssets\Models`
* Open the project, and import the package at `Unity/AirsimAssets`

#### Build it
You can build the AirlibWrapper project using Visual Studio 2017 or msbuild. Make sure you have all the proper Visual Studio dependecies before trying to build. See the AirLib setup instructions [here](https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md) for more details.

### Mac
TO DO. 

