# Airsim Unity

Airsim Unity allows you to run your simulators in the [Unity Engine](https://unity3d.com/). This project comes with some sample Unity projects and a wrapper around the AirLib library to run as a [native plugin](https://docs.unity3d.com/Manual/NativePlugins.html) in Unity. 
Included are two basic Unity Projects, one for a Car simulator and another for a Drone simulator. They are meant to be lightweight, and can be used to verify your setup is correct.
Once you have things working in the basic projects, you can move on to the more detailed Unity projects. They can be found in the [Releases](https://github.com/Microsoft/AirSim/releases) tab. 

## Warning: Beta Sofware
This project is still in early development, expect some rough edges. 

We are working to fully support the full AirLib API and feature set, but some things may be missing. 

## Installation

### Install Unity
Download Unity [here](https://unity3d.com/get-unity/download).

### Run the AirLib setup
Follow the instructions found [here](https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md). You can skip the Unreal steps.

### Windows Steps

#### Download it
You can download a prebuilt version of the Demo project in the releases page. Or you can follow the next section to build it yourself.

#### Build it
Run the `build.cmd` script to install the neccesary components before you start your Unity project. 

### Start the Unity Project
Start Unity and click Open project. Select the folder UnityDemo, and then hit the button `SelectFolder`. A Unity project will open up with default scene `SimModeSelector`.

Hit the play button, and the Car demo will start. If you want to load the Drone demo, you can select the scene `DroneDemo`, or you can change the SimMode to be Multirotor in your Settings.json file.
You can read more about Settings.json [here](https://github.com/Microsoft/AirSim/blob/master/docs/settings.md).

### Linux/Mac Steps
This is currently unsupported, but we hope to add support for this in the future. This would involved building the AirlibWrapper to a plugin format as described [here](https://docs.unity3d.com/Manual/PluginsForDesktop.html).

In the meantime, you can setup your simulator in the Windows Unity editor, and build for your desired platform.

## Running Airsim

See the details [here](https://github.com/Microsoft/AirSim/blob/master/docs/apis.md) on how to interact with your Airsim simulation using the API's.  

## Acknowledgement

The drone object was provided by user 31415926 on [sketchfab](https://sketchfab.com/models/055841df0fb24cd4abde06a91f7d360a). It is licensed under the [CC License](https://creativecommons.org/licenses/by/4.0/).

The rest of this project follows the same License as the Airsim project.
