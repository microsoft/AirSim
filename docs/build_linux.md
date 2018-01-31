# Build AirSim on Linux

The current recommended and tested environment is **Ubuntu 16.04 LTS**. Theoretically, you can build on other distros and OSX as well, but we haven't tested it.

## Install and Build

It's super simple: 1-2-3!

1. Make sure you are [registered with Epic Games](https://docs.unrealengine.com/latest/INT/Platforms/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/1/index.html). This is required to get source code access for Unreal Engine.
2. Clone Unreal in your favorite folder and build it (this may take a while!). **Note**: We only support Unreal 4.17 at present.
   ```bash
   # go to the folder where you clone GitHub projects
   git clone -b 4.17 https://github.com/EpicGames/UnrealEngine.git
   cd UnrealEngine
   # the Unreal build was broken a few times so we will get the commit that works
   git checkout af96417313a908b20621a443175ba91683c238c8
   ./Setup.sh
   ./GenerateProjectFiles.sh
   make
   ```
3. Clone AirSim and build it:
   ```bash
   # go to the folder where you clone GitHub projects
   git clone https://github.com/Microsoft/AirSim.git
   cd AirSim
   ./setup.sh
   ./build.sh
   ```

## Setup Remote Control

A remote control is required if you want to fly manually. See the [remote control setup](remote_control.md) for more details.

Alternatively, you can use [APIs](apis.md) for programmatic control or use the so-called [Computer Vision mode](image_apis.md) to move around using the keyboard.

## Setup Unreal Environment

Finally, you will need an Unreal project that hosts the environment for your vehicles. AirSim comes with a built-in "Blocks Environment" which you can use, or you can create your own. Please see [setting up Unreal Environment](unreal_proj.md).

## FAQ

#### What are the known issues with Unreal 4.16?

* One of the major issues is [this bug in Unreal](https://answers.unrealengine.com/questions/664905/unreal-crashes-on-two-lines-of-extremely-simple-st.html). We have a workaround for some parts of the code but we haven't tested if everything is covered.
* Clicking the "End" button causes Unreal to crash.
* The report function (when you press <kbd>R</kbd>) also causes a crash because of the above reasons.

#### What are the known issues with Unreal 4.17?

* We have seen some random crashes during the startup.
* You might get a warning that says that the AirSim plugin is incompatible, which you can ignore.
* Clicking the "End" button freezes the Unreal Editor. When this happens you will need to manually kill the process.

#### Unreal crashed! How do I know what went wrong?

Go to the `MyUnrealProject/Saved/Crashes` folder and search for the file `MyProject.log` within its subdirectories. At the end of this file you will see the stack trace and messages. You can also take a look at the `Diagnostics.txt` file.

#### How do I use an IDE on Linux?

You can use Qt Creator or CodeLite. Instructions for Qt Creator are available [here](https://docs.unrealengine.com/latest/INT/Platforms/Linux/BeginnerLinuxDeveloper/SettingUpAnIDE/index.html).

#### Can I cross compile for Linux from a Windows machine?

Yes, you can, but we haven't tested it. You can find the instructions [here](https://docs.unrealengine.com/latest/INT/Platforms/Linux/GettingStarted/index.html).

#### What compiler and stdlib does AirSim use?

We use the same compiler, **Clang 3.9**, and stdlib, **libc++**, that Unreal uses. AirSim's `setup.sh` will automatically download them both. The libc++ source code is cloned into the `llvm-source` folder and is built into the `llvm-build` folder, from where CMake uses libc++.

#### Can I use AirSim with Unreal 4.16?

Yes! The `*.Build.cs` files are, however, no longer compatible (you will get a compile error). You can find files for 4.16 as `*.Build.4.16.cs` so just rename those.

#### What version of CMake does the AirSim build use?

3.5.0 or higher. This should be the default in Ubuntu 16.04. You can check your CMake version using `cmake --version`. If you have an older version, follow [these instructions](cmake_linux.md) or see the [CMake website](https://cmake.org/install/).

#### Can I compile AirSim in BashOnWindows?

Yes, however, you can't run Unreal from BashOnWindows. So this is kind of useful to check a Linux compile, but not for an end-to-end run. See the [BashOnWindows install guide](https://msdn.microsoft.com/en-us/commandline/wsl/install_guide). Make sure to have the latest version (Windows 10 Creators Edition) as previous versions had various issues. Also, don't invoke `bash` from `Visual Studio Command Prompt`, otherwise CMake might find VC++ and try and use that!

#### Where can I find more info on running Unreal on Linux?

* Start here: [Unreal on Linux](https://docs.unrealengine.com/latest/INT/Platforms/Linux/index.html)
* [Building Unreal on Linux](https://wiki.unrealengine.com/Building_On_Linux#Clang)
* [Unreal Linux Support](https://wiki.unrealengine.com/Linux_Support)
* [Unreal Cross Compilation](https://wiki.unrealengine.com/Compiling_For_Linux)
