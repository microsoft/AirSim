# Build AirSim on Windows

## Install Unreal Engine

1. [Download](https://www.unrealengine.com/download) the Epic Games Launcher. While the Unreal Engine is open source and free to download, registration is still required.
2. Run the Epic Games Launcher, open the Library tab from left, click on the "Add Versions" which should show the option to download Unreal 4.18 as shown below. If you have multiple versions of Unreal installed then make sure 4.18 is "Current" by clicking down arrow next to the Launch button for the version.

   **Note**: If you have UE 4.16 or older projects, please see the [upgrade guide](unreal_upgrade.md) to upgrade your projects.

## Build AirSim

  1. You will need Visual Studio 2017 (make sure to install VC++ and Windows SDK).  Ensure that CMake tools for C++ are installed.
  2. Install CMake from https://cmake.org/download/
  3. Start `x64 Native Tools Command Prompt for VS 2017`. Create a folder for the repo and run `git clone https://github.com/Microsoft/AirSim.git`.
  4. Run `build.cmd` from the command line. This will create ready to use plugin bits in the `Unreal\Plugins` folder that can be dropped into any Unreal project.
  5. Copy the 'Unreal\Plugins' folder to the 'Unreal\Environments\Blocks' folder.


## Setup Remote Control

A remote control is required if you want to fly manually. See the [remote control setup](remote_control.md) for more details.

Alternatively, you can use [APIs](apis.md) for programmatic control or use the so-called [Computer Vision mode](image_apis.md) to move around using the keyboard.

## Setting up API Control

To establish basic programmatic / API control, you must first start Unreal Engine Editor, then load and start your environment, and then attempt to control the simulation.

  1. Start the Epic Games Launcher
  2. Click the "Launch Unreal Engine 4.18.3" button  (your ".3" part of the version number may be different).  This will cause the Unreal Engine Editor to run, which will want you to load a project.
  3. Select the 'Projects' tab, then click the 'Browse' button, navigate to '...\AirSim\Unreal\Environments\Blocks' and select 'Blocks.uproject'.
  4. To ensure that the framerate remains good while another window (such as your code's window) has focus, go to 'Edit->Editor Preferences', in the 'Search' box type 'CPU' and ensure that the 'Use Less CPU when in Background' is UNCHECKED.
  5. Click the 'Play' button.  This will cause the simulation to run.  It will prompt you to select run a car or quadrotor simulation.  For this example select the option that causes the quadrotor simulation to be started.  This will cause a 'settings.json' to be created in your 'Documents' folder on your computer with a default (mostly empty) version. 
  6. Click the 'Stop' button in Unreal Engine Editor to cause the simulation to end.  Open the 'Documents\AirSim\settings.json' file and set it to:

    {
      "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
      "SettingsVersion": 1.0,
      "SimMode": "Multirotor",
      "RpcEnabled": true,
    
      "Vehicles": {
        "SimpleFlight": {
          "VehicleType": "SimpleFlight",
    
          "AllowAPIAlways": true
        }
      }

  7. Hit the 'Play' button again, which will use these new settings.
  
Assuming you have python installed already (anaconda):

  1. Open anaconda prompt and navigate to the 'AirSim\PythonClient\multirotor' directory.
  2. Install the remote procedure call library with 'pip install msgpack-python'
  3. Install the AirSim library with 'pip install airsim'
  4. Run a simple example.  'python hello_drone.py'

## Setup Unreal Environment

Finally, you will need an Unreal project that hosts the environment for your vehicles. AirSim comes with a built-in "Blocks Environment" which you can use, or you can create your own. Please see [setting up Unreal Environment](unreal_proj.md).

## FAQ

#### I get error "'corecrt.h': No such file or directory"
Very likely you don't have [Windows SDK](https://developercommunity.visualstudio.com/content/problem/3754/cant-compile-c-program-because-of-sdk-81cant-add-a.html) installed with Visual Studio. 

#### How do I use PX4 firmware with AirSim?
By default, AirSim uses its own built-in firmware called [simple_flight](simple_flight.md). There is no additional setup if you just want to go with it. If you want to switch to using PX4 instead then please see [this guide](px4_setup.md).

#### I made changes in Visual Studio but there is no effect

Sometimes the Unreal + VS build system doesn't recompile if you make changes to only header files. To ensure a recompile, make some Unreal based cpp file "dirty" like AirSimGameMode.cpp.

#### Unreal still uses VS2015 or I'm getting some link error
Running serveral versions of VS can lead to issues when compiling UE projects. One problem that may arise is that UE will try to compile with an older version of VS which may or may not work. There are two settings in Unreal, one for for the engine and one for the project, to adjust the version of VS to be used.
1. Edit -> Editor preferences -> General -> Source code
2. Edit -> Project Settings -> Platforms -> Windows -> Toolchain ->CompilerVersion

In some cases, these settings will still not lead to the desired result and errors such as the following might be produced: LINK : fatal error LNK1181: cannot open input file 'ws2_32.lib'

To resolve such issues the following procedure can be applied:
1. Uninstall all old versions of VS using the [VisualStudioUninstaller](https://github.com/Microsoft/VisualStudioUninstaller/releases)
2. Repair/Install VS2017
3. Restart machine and install Epic launcher and desired version of the engine
