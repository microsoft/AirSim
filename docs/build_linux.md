# Build AirSim on Linux

The current recommended and tested environment is **Ubuntu 16.04 LTS**. Theoretically, you can build on other distros and OSX as well, but we haven't tested it.

We've two options - you can either build inside docker containers or your host machine. 

## Docker 
Please see instructions [here](https://github.com/madratman/AirSim/blob/PR/docker_ubuntu/docs/docker_ubuntu.md)

## Host machine

### Build Unreal Engine and Airsim
- Make sure you are [registered with Epic Games](https://docs.unrealengine.com/latest/INT/Platforms/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/1/index.html). This is required to get source code access for Unreal Engine.

- Clone Unreal in your favorite folder and build it (this may take a while!). **Note**: We only support Unreal 4.18 at present.
   ```bash
   # go to the folder where you clone GitHub projects
   git clone -b 4.18 https://github.com/EpicGames/UnrealEngine.git
   cd UnrealEngine
   ./Setup.sh
   ./GenerateProjectFiles.sh
   make
   ```

- Clone AirSim and build it:
   ```bash
   # go to the folder where you clone GitHub projects
   git clone https://github.com/Microsoft/AirSim.git
   cd AirSim
   ./setup.sh
   ./build.sh
   ```

### Build Unreal Environment

Finally, you will need an Unreal project that hosts the environment for your vehicles. AirSim comes with a built-in "Blocks Environment" which you can use, or you can create your own. Please see [setting up Unreal Environment](unreal_proj.md).

### [Optional] Setup Remote Control (Multirotor Only)

A remote control is required if you want to fly manually. See the [remote control setup](remote_control.md) for more details.

Alternatively, you can use [APIs](apis.md) for programmatic control or use the so-called [Computer Vision mode](image_apis.md) to move around using the keyboard.

## How to Use AirSim

Once AirSim is set up by following above steps, you can,

- Go to `UnrealEngine` folder and start Unreal by running `UnrealEngine/Engine/Binaries/Linux/UE4Editor`.
- When Unreal Engine prompts for opening or creating project, select Browse and choose `AirSim/Unreal/Environments/Blocks` (or your [custom](unreal_custenv.md) Unreal project).
- If you get prompts to convert project, look for More Options or Convert-In-Place option. If you get prompted to build, chose Yes. If you get prompted to disable AirSim plugin, choose No.
- After Unreal Editor loads, press Play button. Tip: go to 'Edit->Editor Preferences', in the 'Search' box type 'CPU' and ensure that the 'Use Less CPU when in Background' is unchecked.

See [Using APIs](apis.md) and [settings.json](settings.md) for various options available.

## FAQs

- I'm getting error "<MyProject> could not be compiled. Try rebuilding from source manually".
  * This could either happen because of compile error or the fact that your gch files are outdated. Look in to your console window. Do you see something like below?
   ```
   fatal error: file  '/usr/include/linux/version.h''/usr/include/linux/version.h'  has  been  modified  since  the  precompiled  header
   ```
  * If this is the case then look for *.gch file(s) that follows after that message, delete them and try again. Here's [relevant thread](https://answers.unrealengine.com/questions/412349/linux-ue4-build-precompiled-header-fatal-error.html) on Unreal Engine forums.
  * If you see other compile errors in console then open up those source files and see if it is due to changes you made. If not, then report it as issue on GitHub.

- Unreal crashed! How do I know what went wrong?
  * Go to the `MyUnrealProject/Saved/Crashes` folder and search for the file `MyProject.log` within its subdirectories. At the end of this file you will see the stack trace and messages.    
   You can also take a look at the `Diagnostics.txt` file.

- How do I use an IDE on Linux?
  * You can use Qt Creator or CodeLite. Instructions for Qt Creator are available [here](https://docs.unrealengine.com/latest/INT/Platforms/Linux/BeginnerLinuxDeveloper/SettingUpAnIDE/index.html).

- Can I cross compile for Linux from a Windows machine?
  * Yes, you can, but we haven't tested it. You can find the instructions [here](https://docs.unrealengine.com/latest/INT/Platforms/Linux/GettingStarted/index.html).

- What compiler and stdlib does AirSim use?
  * We use the same compiler that Unreal Engine uses, **Clang 5.0**, and stdlib, **libc++**. AirSim's `setup.sh` will automatically download them both. The libc++ source code is cloned into the `llvm-source-(version)` folder and is built into the `llvm-build` folder, from where CMake uses libc++.

- What version of CMake does the AirSim build use?
  * 3.9.0 or higher. This is *not* the default in Ubuntu 16.04 so setup.sh installs it for you. You can check your CMake version using `cmake --version`. If you have an older version, follow [these instructions](cmake_linux.md) or see the [CMake website](https://cmake.org/install/).

- Can I compile AirSim in BashOnWindows?
  * Yes, however, you can't run Unreal from BashOnWindows. So this is kind of useful to check a Linux compile, but not for an end-to-end run.    
   See the [BashOnWindows install guide](https://msdn.microsoft.com/en-us/commandline/wsl/install_guide).   
   Make sure to have the latest version (Windows 10 Creators Edition) as previous versions had various issues.    
   Also, don't invoke `bash` from `Visual Studio Command Prompt`, otherwise CMake might find VC++ and try and use that!

- Where can I find more info on running Unreal on Linux?
   * Start here: [Unreal on Linux](https://docs.unrealengine.com/latest/INT/Platforms/Linux/index.html)
   * [Building Unreal on Linux](https://wiki.unrealengine.com/Building_On_Linux#Clang)
   * [Unreal Linux Support](https://wiki.unrealengine.com/Linux_Support)
   * [Unreal Cross Compilation](https://wiki.unrealengine.com/Compiling_For_Linux)
