# Build It on Linux

The current recommended and tested environment is **Ubuntu 16.04 LTS**. Theoretically you can build on other distros and OSX as well but we haven't tested it.

## Install and Build

It's super simple 1-2-3!

1. Make sure you are [registered with Epic Games](https://docs.unrealengine.com/latest/INT/Platforms/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/1/index.html). This is required to get source code access for Unreal engine.
2. Clone Unreal in your favorite folder and build it (this may take a while!). Note: We only support Unreal 4.17 at present.
```
     # go to folder where you clone GitHub projects
     git clone -b 4.17 https://github.com/EpicGames/UnrealEngine.git
     cd UnrealEngine
     # few times Epic folks broke the build so we will get commit that works
     git checkout af96417313a908b20621a443175ba91683c238c8
     ./Setup.sh
     ./GenerateProjectFiles.sh
     make
```
3. Clone AirSim and build it:
```
     # go to folder where you clone GitHub projects
     git clone https://github.com/Microsoft/AirSim.git
     cd AirSim
     ./setup.sh
     ./build.sh
```

## Setup Remote Control

Remote control is required if you want to fly manually. See [remote control setup](remote_control.md) for more details. Alternatively you can use [APIs](apis.md) for programmatic control or use so-called [Computer Vision mode](image_apis.md) to move around using keyboard.

## Setup Unreal Environment

Finally, you need Unreal project that hosts the environment for your vehicles. AirSim comes with built-in "Blocks Environment" or you can create your own. Please see [setting up Unreal Environment](unreal_proj.md).


## FAQ

#### What are the known issues with Unreal 4.16?

One of the major issue is [this bug in Unreal](https://answers.unrealengine.com/questions/664905/unreal-crashes-on-two-lines-of-extremely-simple-st.html). We have done a workaround for some parts of the code but we haven't tested if everything is covered. Another known issue is that when clicking "End" button causes Unreal to crash. A final known issue is that report function (when you press `R` key), also causes crash because of above reasons.

#### What are the known issues with Unreal 4.17?

At the time of writing (July, 2017), version 4.17 is still in "preview" stage and is unstable. We have seen some rare but random crashes during startup. You might get warning that AirSim plugin is incompatible which you can ignore. Also, when clicking on "End" button freezes the Unreal Editor requiring to manual kill of its process. 

#### Unreal crashed! How do I know what went wrong?

First go to folder `MyUnrealProject/Saved/Crashes` and then search directories for MyProject.log file. At the end of this file you will see stack trace and message. Also see `Diagnostics.txt` file.

#### How do I use IDE in Linux?

You can use Qt or CodeLite. Instructions for Qt Creator is [available here](https://docs.unrealengine.com/latest/INT/Platforms/Linux/BeginnerLinuxDeveloper/SettingUpAnIDE/index.html).

#### Can I cross compile for Linux from Windows machine?

Yes, you can but we haven't tested it. You can find [instructions here](https://docs.unrealengine.com/latest/INT/Platforms/Linux/GettingStarted/index.html).

#### What compiler and stdlib AirSim uses?

We use same compiler that Unreal uses which is Clang 3.9. AirSim's `setup.sh` will automatically download Clang 3.9. We also need to use libc++ that Unreal uses. The libc++ code is cloned by AirSim's `setup.sh` in to `llvm-source` folder and built in `llvm-build` folder. The cmake is the instructed to use libc++ from `llvm-build` folder. For other flavors of Linux and more info, please see [http://apt.llvm.org/](http://apt.llvm.org/).

#### Can use AirSim with Unreal 4.16?

Yes! The `*.Build.cs` files are, however, no longer compatible (you will get compile error). You can find files for 4.16 as `*.Build.4.16.cs` so just rename those. 

#### What CMake version AirSim build uses?

3.5.0 or higher. This should be default in Ubuntu 16.04. You can check your cmake version using `cmake --version`. If you have older version the follow [these instructions](cmake_linux.md) or [see cmake website](https://cmake.org/install/).

#### Can I compile AirSim in BashOnWindows?

Yes, however you can't run Unreal from BashOnWindows. So this is kind of useful to check Linux compile, not for end-to-end run. See [BashOnWindows install guide](https://msdn.microsoft.com/en-us/commandline/wsl/install_guide). Make sure to have latest version (Windows 10 Creators Edition) as previous versions had various issues. Also don't invoke `bash` from `Visual Studio Command Prompt` otherwise cmake might find VC++ and try and use that!

#### I made change in Visual Studio but there is no effect

Sometime Unreal + VS build system don't do recompile if you change only header file. So try making some cpp file dirty.

#### Where can I find more info on running Unreal on Linux?

* [Start here - Unreal on Linux](https://docs.unrealengine.com/latest/INT/Platforms/Linux/index.html)
* [Building Unreal on Linux](https://wiki.unrealengine.com/Building_On_Linux#Clang)
* [Unreal Linux Support](https://wiki.unrealengine.com/Linux_Support)
* [Unreal Cross Compilation](https://wiki.unrealengine.com/Compiling_For_Linux)

