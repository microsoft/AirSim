# Build It on Windows

## Install Unreal Engine

Download Epic Games Launcher from [unreal.com](https://www.unrealengine.com/download). While the Unreal Engine is open source and free to download, registration is still required. Run Epic Games Launcher, switch to Library tab on the left, click on the "Add Versions" which should show the option to download Unreal 4.16 as shown below. If you have multiple versions of Unreal installed then make sure 4.16 is "Current" by clicking down arrow next to the Launch button for the version.

  **Note**: Older versions of Unreal is not supported. Please see [guide to upgrade](unreal_upgrade.md) your projects.

  ![Unreal Versions](images/unreal_versions.png)
  
## Build AirSim

  1. You need Visual Studio 2015 Update 3 (make sure to install VC++) or newer. Other versions haven't been tested.
  2. Start VS2015 x64 Native Tools Command Prompt. Create a folder for repo and run `git clone https://github.com/Microsoft/AirSim.git`.
  3. Install [cmake](https://cmake.org/download/) which is used to build the rpclib submodule.
  4. Run `build.cmd` from the command line. This will create ready to use plugin bits in `Unreal\Plugins` folder that can be dropped in to any Unreal projects.

## Setup Remote Control

Remote control is required if you want to fly manually. See [remote control setup](remote_control.md) for more details. Alternatively you can use [APIs](docs/apis.md) for programmatic control or use so-called [Computer Vision mode](image_apis.md) to move around using keyboard.

## Setup Unreal Environment

Finally, you need Unreal project that hosts the environment for your vehicles. AirSim comes with built-in "Blocks Environment" or you can create your own. Please see [setting up Unreal Environment](unreal_proj.md).

## FAQ

#### How do I use PX4 firmware with AirSim?
By default, AirSim uses its own built-in firmware called [simple_flight](simple_flight.md). There is no additional setup if you just want to go with it. If you want to switch to using PX4 instead then please see [this guide](px4_setup.md).

#### Build is not working on VS 2017
Known working config is:
````
Windows 10 (Education) x64
VS2015 update 3 (x86) with VC++
Cmake 3.7 (x86)
````
Even though cmake 3.7 says it added support for VS 2017 folks are reporting build issues with that.

