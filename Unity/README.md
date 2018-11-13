# Airsim Unity

* Airsim Unity allows you to run your simulators in the [Unity Engine](https://unity3d.com/). This project comes with some sample Unity projects and a wrapper around the AirLib library to run as a [native plugin](https://docs.unity3d.com/Manual/NativePlugins.html) in Unity. 
* Included are two basic Unity Projects, one for a Car simulator and another for a Drone simulator. They are meant to be lightweight, and can be used to verify your setup is correct. 
* Check out the [Unity blog](https://blogs.unity3d.com/2018/11/14/airsim-on-unity-experiment-with-autonomous-vehicle-simulation/).   

### Warning: Beta Sofware
This project is still in early development, expect some rough edges.   
We are working to fully support the full AirLib API and feature set, but some things may be missing. 

## Windows

### Binaries
You can download a prebuilt version of the Demo project in the releases page. 

### Building from source

#### Install Unity
* [Download Unity](https://unity3d.com/get-unity/download) via the **Unity Hub**. 
* Install Unity using the Unity Hub by following [the instructions here](https://docs.unity3d.com/Manual/GettingStartedInstallingHub.html). 
* Note: If you are using Unity for the first time, check out [the Getting started guide](https://docs.unity3d.com/Manual/GettingStarted.html). The [Unity User Manual](https://docs.unity3d.com/Manual/UnityManual.html) has additional tips, resources, and FAQs.

#### Build Unity Project
* `cd` to the [Unity folder](https://github.com/Microsoft/AirSim/tree/master/Unity) and run `build.cmd` script therein. 
* Additionally, there is a free environment `Windridge City` which you can download from Unity Asset Store, which is perfect for AirSim experimentation. And, of course, you can always create your own environment.

#### Usage 
* Start Unity and click `Open project`. Select the folder `AirSim\Unity\UnityDemo`, and then hit the button `Select Folder`. A Unity project will open up with default scene `SimModeSelector`. (
Make sure that *SimModeSelector* scene is loaded.)
* Hit the play button, and the Car demo will start. 
* If you want to load the Drone demo, you can select the scene `DroneDemo` in `Assets -> Scenes` in the pane at the bottom.  
Alternatively, you can change the SimMode to be Multirotor in your Settings.json file. (You can read more about Settings.json [here](https://github.com/Microsoft/AirSim/blob/master/docs/settings.md))
* Use WASD/Arrow keys or AirSim client to control the car and PageUP/PageDown with WASD/Arrow keys to control drone.
* Keys 0, 1, 2, and 3 are used to toggle windows of different camera views.
* Press *Record* button(Red button) located at the right bottom corner of the screen, to toggle recording of the simulation data. The recorded data can be found at **Documents\AirSim\(Date of recording)**

#### Using Airsim API
See the details [here](https://github.com/Microsoft/AirSim/blob/master/docs/apis.md) on how to interact with your Airsim simulation using the APIs.  

### Acknowledgements
* The drone object was provided by user 31415926 on [sketchfab](https://sketchfab.com/models/055841df0fb24cd4abde06a91f7d360a). It is licensed under the [CC License](https://creativecommons.org/licenses/by/4.0/).
* The rest of this project follows the same License as the Airsim project.
