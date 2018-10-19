# Airsim Unity

Airsim Unity allows you to run your simulators in the [Unity Engine](https://unity3d.com/). This project comes with some sample Unity projects and a wrapper around the AirLib library to run as a [native plugin](https://docs.unity3d.com/Manual/NativePlugins.html) in Unity.

## Installation

### Install Unity
We recommend using 2018.1 or later, and using the Unity hub. They can be downloaded [here](https://unity3d.com/get-unity/download).  

### Pick a Unity Project
This repository comes with two basic Unity Projects, one for a Car simulator and another for a Drone simulator. They are meant to be lightweight, and can be used to verify your setup is correct.

Once you have things working in the basic projects, you can move on to the more detailed Unity projects. They can be found in the [Releases](https://github.com/Microsoft/AirSim/releases) tab. 

### Run the AirLib setup
Follow the instructions found [here](https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md). You can skip the Unreal steps.

### Windows Steps

#### Download it
* Pick a project of your choice. 
* Download the `AirLibWrapper.dll` at the [Releases](https://github.com/Microsoft/AirSim/releases) page.
* If you are running a Drone project, download the `drone.obj` from the Releases page and place at path `Unity\DroneDemo\Assets\AirSimAssets\Models`
* Setup your `Settings.json` file as described [here](https://github.com/Microsoft/AirSim/blob/master/docs/settings.md)
* Open the project, and import the package at `Unity/AirsimAssets`

#### Build it
You can build the AirlibWrapper project using Visual Studio 2017 or msbuild. Make sure you have all the proper Visual Studio dependencies before trying to build. See the AirLib setup instructions [here](https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md) for more details.

We recommend setting your configuration to `Release` for better performance. Make sure to pick the right platform for your build (for example, x64).

### Linux/Mac Steps
This is currently unsupported, but we hope to add support for this in the future. This would involved building the AirlibWrapper to a plugin format as described [here](https://docs.unity3d.com/Manual/PluginsForDesktop.html).

In the meantime, you can setup your simulator in the Windows Unity editor, and build for your desired platform.

## Running Airsim

See the details [here](https://github.com/Microsoft/AirSim/blob/master/docs/apis.md) on how to interact with your Airsim simulation using the API's.  

