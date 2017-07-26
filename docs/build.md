# How To Build and Install
**Make sure you have read the [prerequisites](prereq.md).**

## Install Unreal Engine
  1. Download Epic Games Launcher from [unreal.com](https://www.unrealengine.com/download). While the Unreal Engine is open source, cross platform and free to download, registration is still required. Run Epic Games Launcher, switch to Library tab on the left, click on the "Add Versions" which should show the option to download Unreal 4.16 as shown below.

  **Note**: Older versions of Unreal is not supported. Some tips to upgrade your projects can be found [here](unreal_upgrade.md).

  ![Unreal Versions](images/unreal_versions.png)
  
## Get the Code and Build

### Windows

  1. You need Visual Studio 2015 Update 3 (make sure to install VC++). Other versions haven't been tested.
  2. Start VS2015 x64 Native Tools Command Prompt. Create a folder for repo and run
  `git clone https://github.com/Microsoft/AirSim.git`
  3. Install [cmake](https://cmake.org/download/) which is used to build the rpclib submodule.
  4. Run `build.cmd` from the command line. If everything goes ok, it will copy all the binaries that you need to Unreal/Plugins folder in your repo. 
  This Plugins folder can then be simply copied to your Unreal environment.

### Linux

  1. See [Linux build](linux_build.md).
  
## Create Unreal Environment Project

Please go to [Unreal Demo Setup](unreal_demo.md) for full setup steps to get AirSim working in the Landscape Mountains environment.


### FAQ

If you run into problems, check the [FAQ](faq.md) and feel free to post issues on the [AirSim github](https://github.com/Microsoft/AirSim/issues).
