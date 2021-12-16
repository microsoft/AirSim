# Build AirSim on Linux & MacOS

The current recommended and tested environment is **Ubuntu 18.04 LTS**. Theoretically, you can build on other distros as well, but we haven't tested it.

Only macOS **Catalina (10.15)** is supported.

We've two options - you can either build inside docker containers or your host machine.

## Docker

Please see instructions [here](docker_ubuntu.md)

## Host machine

### Pre-build Setup

#### Linux - Build Unreal Engine

- Make sure you are [registered with Epic Games](https://docs.unrealengine.com/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/index.html). This is required to get source code access for Unreal Engine.

- Clone Unreal in your favorite folder and build it (this may take a while!). **Note**: We only support Unreal >= 4.25 at present. We recommend using 4.25.

```bash
# go to the folder where you clone GitHub projects
git clone -b 4.25 git@github.com:EpicGames/UnrealEngine.git
cd UnrealEngine
./Setup.sh
./GenerateProjectFiles.sh
make
```

#### macOS - Download Unreal Engine

1. [Download](https://www.unrealengine.com/download) the Epic Games Launcher. While the Unreal Engine is open source and free to download, registration is still required.
2. Run the Epic Games Launcher, open the `Library` tab on the left pane.
Click on the `Add Versions` which should show the option to download **Unreal 4.25** as shown below. If you have multiple versions of Unreal installed then **make sure 4.25 is set to `current`** by clicking down arrow next to the Launch button for the version.

   **Note**: AirSim also works with UE >= 4.24, however, we recommend 4.25.
   **Note**: If you have UE 4.16 or older projects, please see the [upgrade guide](unreal_upgrade.md) to upgrade your projects.

### Build AirSim

- Clone AirSim and build it:

```bash
# go to the folder where you clone GitHub projects
git clone https://github.com/Microsoft/AirSim.git
cd AirSim
```

  By default AirSim uses clang 8 to build for compatibility with UE 4.25. The setup script will install the right version of cmake, llvm, and eigen.

```bash
./setup.sh
./build.sh
# use ./build.sh --debug to build in debug mode
```

### Build Unreal Environment

Finally, you will need an Unreal project that hosts the environment for your vehicles. AirSim comes with a built-in "Blocks Environment" which you can use, or you can create your own. Please see [setting up Unreal Environment](unreal_proj.md) if you'd like to setup your own environment.

## How to Use AirSim

### Linux

Once AirSim is setup:

- Go to `UnrealEngine` installation folder and start Unreal by running `./Engine/Binaries/Linux/UE4Editor`.
- When Unreal Engine prompts for opening or creating project, select Browse and choose `AirSim/Unreal/Environments/Blocks` (or your [custom](unreal_custenv.md) Unreal project).
- Alternatively, the project file can be passed as a commandline argument. For Blocks: `./Engine/Binaries/Linux/UE4Editor <AirSim_path>/Unreal/Environments/Blocks/Blocks.uproject`
- If you get prompts to convert project, look for More Options or Convert-In-Place option. If you get prompted to build, choose Yes. If you get prompted to disable AirSim plugin, choose No.
- After Unreal Editor loads, press Play button.

### Mac

- Browse to `AirSim/Unreal/Environments/Blocks`.
- Run `./GenerateProjectFiles.sh <UE_PATH>` from the terminal, where `UE_PATH` is the path to the Unreal installation folder. (By default, this is `/Users/Shared/Epic\ Games/UE_4.25/`) The script creates an XCode workspace by the name Blocks.xcworkspace.
- Open the XCode workspace, and press the Build and run button in the top left.
- After Unreal Editor loads, press Play button.

See [Using APIs](apis.md) and [settings.json](settings.md) for various options available for AirSim usage.

!!! tip
    Go to 'Edit->Editor Preferences', in the 'Search' box type 'CPU' and ensure that the 'Use Less CPU when in Background' is unchecked.

### [Optional] Setup Remote Control (Multirotor Only)

A remote control is required if you want to fly manually. See the [remote control setup](remote_control.md) for more details.

Alternatively, you can use [APIs](apis.md) for programmatic control or use the so-called [Computer Vision mode](image_apis.md) to move around using the keyboard.
