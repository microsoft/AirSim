# How To Build and Install
**Make sure you have read the [prerequisites](docs/prereq.md).**

## Install Unreal Engine
  1. Download Unreal 4 engine from [unreal.com](https://www.unrealengine.com/dashboard). While Unreal Engine is open source, cross platform and free to download, registration is still required as of this writing. You need version 4.14 or higher.
  2. After download, run Epic Game Launcher and click the big yellow "Install" button under Unreal Engine tab. 
 
  ![Epic launcher install](images/epic_launcher_install.png).

## Install Dependencies
AirSim code has two external dependencies: Eigen library and Boost library.
1. If you don't have already, [install Boost](docs/install_boost.md).
2. If you don't have already, [install Eigen](docs/install_eigen.md).

## Get the Code and Build
  1. You need Visual Studio 2015 Update 3 (make sure to install VC++). Other versions haven't been tested.
  2. Start VS2015 x64 Native Tools Command Prompt. Create a folder for repo and run
  `git clone https://github.com/Microsoft/AirSim.git`
  3. Run `build`. If everything goes fine, you should have all binaries you need.

## Create Unreal Environment Project
To run the simulator, you need environment and its very easy to create one! [Unreal Marketplace](https://www.unrealengine.com/marketplace) has dozens of prebuilt extra-ordinarily detailed [environments](https://www.unrealengine.com/marketplace/content-cat/assets/environments) ranging from Moon to Mars and everything in between. The one we have used for testing our code is called [Modular Neighborhood Pack](https://www.unrealengine.com/marketplace/modular-neighborhood-pack) but you can use any environment.
  1. Either purchase an environment from Unreal Marketplace or choose one of the free ones such as [Infinity Blade series](https://www.unrealengine.com/marketplace/infinity-blade-plain-lands). Alternatively, if you look under the Learn tab in Epic Game Launcher, you will find many free samples you can use. One of our favorite is, of course, "A Boy and His Kite" which is 100 square mile of highly detailed environment (causion: you will need *very* beefy specs to run it!).
  2. Once you have environment, you can simply go to Library tab of Epic Game Launcher and add in to any project you like. We recommand creating a new black C++ project with no Starter Content and add your environment in to it. Tip: if environment comes with MatineeActor, delete it to avoid any demo sequences.

## Install the AirSim Plugin
### Copy plugins folder
  Copy the `Unreal\Plugins` folder from the build you did in above section in to the root of your Unreal project's folder. The overall structure should look something like this: 

  ![Unreal folder structure](images/unreal_folders.png)

### Enable plugin for your Unreal project
  In your Unreal project's .uproject file, add Plugins section and key *"AdditionalDependencies"* so your project file looks like this:
  ```
    {
        "FileVersion": 3,
        "EngineAssociation": "4.14",
        "Category": "",
        "Description": "",
        "Modules": [
            {
                "Name": "MyUnrealProject",
                "Type": "Runtime",
                "LoadingPhase": "Default",
                "AdditionalDependencies": [
                    "AirSim"
                ]
            }
        ],
        "Plugins": [
            {
                "Name": "AirSim",
                "Enabled": true
            }
        ]
    }  
  ```

### Ready, Set, Go!
You are all ready to go now! 

1. Right click on Unreal project .uproject file, then Generate Visual Studio Project files. 
2. Double click on .sln file to open the solution. 
3. Hit F5.
4. After Unreal Editor comes up select SimGameMode in World settings.
5. Make sure your environment has Player Start component or add one. This is where drone will be placed.
6. Hit Play button.

Congratulations! You are now running AirSim in your own Unreal environment.