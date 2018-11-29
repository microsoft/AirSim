# AirSim on Unity

* AirSim on Unity allows you to run your simulators in the [Unity Engine](https://unity3d.com/). This project comes with some sample Unity projects and a wrapper around the AirLib library to run as a [native plugin](https://docs.unity3d.com/Manual/NativePlugins.html) in Unity. 
* Included are two basic Unity Projects, one for a Car simulator and another for a Drone simulator. They are meant to be lightweight, and can be used to verify your setup is correct. 
* Check out the [Unity blogpost](https://blogs.unity3d.com/2018/11/14/airsim-on-unity-experiment-with-autonomous-vehicle-simulation/) for overview on the release.  

### Warning: Experimental Release
This project is still in early development, expect some rough edges. We are working to fully support the full AirLib API and feature set, but some things may be missing. 

## Windows
### Building from source
#### Install Unity
* Download **Unity Hub** from [this page](https://unity3d.com/get-unity/download). 
* Install **Unity 2018.2.15f1** using the Unity Hub. [Detailed instructions here](https://docs.unity3d.com/Manual/GettingStartedInstallingHub.html). 
* Note: If you are using Unity for the first time, check out [the Getting started guide](https://docs.unity3d.com/Manual/GettingStarted.html). The [Unity User Manual](https://docs.unity3d.com/Manual/UnityManual.html) has additional tips, resources, and FAQs.

#### Build Airsim
* Install Visual Studio 2017. 
**Make sure** to select **VC++** and **Windows SDK 8.1** while installing VS 2017.   

* Start `x64 Native Tools Command Prompt for VS 2017`. 
* Clone the repo: `git clone https://github.com/Microsoft/AirSim.git`, and go the AirSim directory by `cd AirSim`. 
* Run `build.cmd` from the command line. 

#### Build Unity Project
* Go inside the AirSim\Unity directory: `cd Unity`. 
* Build the unity project: `build.cmd`.   
* Additionally, there is a free environment `Windridge City` which you can download from [Unity Asset Store](https://assetstore.unity.com/packages/3d/environments/roadways/windridge-city-132222). And, of course, you can always create your own environment.

#### Usage 
* Start Unity and click `Open project`. 
* Select the folder `AirSim\Unity\UnityDemo`, and then hit the button `Select Folder`. 
* In the bottom pane, Click on `Projects`->`Assets`->`Scenes`. Then, **Double-click** on `SimModeSelector`, `DroneDemo`, or `CarDemo`. 
* Hit the play button to start the simulation (and hit play again to stop the simulation. .  
* Alternatively, you can change the SimMode in your `Settings.json` file. (You can read more about [`Settings.json` here](https://github.com/Microsoft/AirSim/blob/master/docs/settings.md))
* Controlling the car:    
Use `WASD` or the `Arrow keys` or the AirSim client.   
* Controlling the drone:    
Use the `PageUP`/`PageDown` with` WASD`/`Arrow` keys.
* Changing camera views:    
Keys `0`, `1`, `2`, `3` are used to toggle windows of different camera views.
* Recording simulation data:    
Press *Record* button(Red button) located at the right bottom corner of the screen, to toggle recording of the simulation data. The recorded data can be found at `Documents\AirSim\(Date of recording)`

#### Using Airsim API
* For quickstart with the Python APIs for the car or the drone, simply run the [`hello_car.py`](https://github.com/Microsoft/AirSim/blob/master/PythonClient/car/hello_car.py) or the [`hello_drone.py`](https://github.com/Microsoft/AirSim/blob/master/PythonClient/multirotor/hello_drone.py) script accordingly. 
* Details of the AirSim C++ and Python APIs are [here](https://github.com/Microsoft/AirSim/blob/master/docs/apis.md). 

### Acknowledgements
* The drone object was provided by user 31415926 on [sketchfab](https://sketchfab.com/models/055841df0fb24cd4abde06a91f7d360a). It is licensed under the [CC License](https://creativecommons.org/licenses/by/4.0/).
* The rest of this project follows the same License as the Airsim project.