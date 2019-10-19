# Build AirSim on Windows

## Install Unreal Engine

1. [Download](https://www.unrealengine.com/download) the Epic Games Launcher. While the Unreal Engine is open source and free to download, registration is still required.
2. Run the Epic Games Launcher, open the `Library` tab on the left pane.   
Click on the `Add Versions` which should show the option to download **Unreal 4.18** as shown below. If you have multiple versions of Unreal installed then **make sure 4.18 is set to `current`** by clicking down arrow next to the Launch button for the version.

   **Note**: If you have UE 4.16 or older projects, please see the [upgrade guide](unreal_upgrade.md) to upgrade your projects.

## Build AirSim
* Install Visual Studio 2017.    
**Make sure** to select **VC++** and **Windows SDK 8.1** while installing VS 2017.   
* Start `x64 Native Tools Command Prompt for VS 2017`. 
* Clone the repo: `git clone https://github.com/Microsoft/AirSim.git`, and go the AirSim directory by `cd AirSim`. 
* Run `build.cmd` from the command line. This will create ready to use plugin bits in the `Unreal\Plugins` folder that can be dropped into any Unreal project.

## Build Unreal Project

Finally, you will need an Unreal project that hosts the environment for your vehicles. AirSim comes with a built-in "Blocks Environment" which you can use, or you can create your own. Please see [setting up Unreal Environment](unreal_proj.md).

## Setup Remote Control (Multirotor only)

A remote control is required if you want to fly manually. See the [remote control setup](remote_control.md) for more details.

Alternatively, you can use [APIs](apis.md) for programmatic control or use the so-called [Computer Vision mode](image_apis.md) to move around using the keyboard.

## How to Use AirSim

Once AirSim is set up by following above steps, you can,

1. Double click on .sln file to load the Blocks project in `Unreal\Environments\Blocks` (or .sln file in your own [custom](unreal_custenv.md) Unreal project). If you don't see .sln file then you probably haven't completed steps in Build Unreal Project section above.
2. Select your Unreal project as Start Up project (for example, Blocks project) and make sure Build config is set to "Develop Editor" and x64.
3. After Unreal Editor loads, press Play button. Tip: go to 'Edit->Editor Preferences', in the 'Search' box type 'CPU' and ensure that the 'Use Less CPU when in Background' is unchecked.

See [Using APIs](apis.md) and [settings.json](settings.md) for various options available.

# AirSim on Unity (Experimental)
[Unity](https://unity3d.com/) is another great game engine platform and we have an [experimental release](https://github.com/Microsoft/AirSim/tree/master/Unity) of AirSim on Unity. Please note that this is work in progress and all features may not work yet. 

# FAQ
#### I get `error C100 : An internal error has occurred in the compiler` when running build.cmd
We have noticed this happening with VS version `15.9.0` and have checked-in a workaround in AirSim code. If you have this VS version, please make sure to pull the latest AirSim code.

#### I get error "'corecrt.h': No such file or directory" or "Windows SDK version 8.1 not found"
Very likely you don't have [Windows SDK](https://developercommunity.visualstudio.com/content/problem/3754/cant-compile-c-program-because-of-sdk-81cant-add-a.html) installed with Visual Studio. 

#### How do I use PX4 firmware with AirSim?
By default, AirSim uses its own built-in firmware called [simple_flight](simple_flight.md). There is no additional setup if you just want to go with it. If you want to switch to using PX4 instead then please see [this guide](px4_setup.md).

#### I made changes in Visual Studio but there is no effect

Sometimes the Unreal + VS build system doesn't recompile if you make changes to only header files. To ensure a recompile, make some Unreal based cpp file "dirty" like AirSimGameMode.cpp.

#### Unreal still uses VS2015 or I'm getting some link error
Running several versions of VS can lead to issues when compiling UE projects. One problem that may arise is that UE will try to compile with an older version of VS which may or may not work. There are two settings in Unreal, one for for the engine and one for the project, to adjust the version of VS to be used.
1. Edit -> Editor preferences -> General -> Source code
2. Edit -> Project Settings -> Platforms -> Windows -> Toolchain ->CompilerVersion

In some cases, these settings will still not lead to the desired result and errors such as the following might be produced: LINK : fatal error LNK1181: cannot open input file 'ws2_32.lib'

To resolve such issues the following procedure can be applied:
1. Uninstall all old versions of VS using the [VisualStudioUninstaller](https://github.com/Microsoft/VisualStudioUninstaller/releases)
2. Repair/Install VS2017
3. Restart machine and install Epic launcher and desired version of the engine