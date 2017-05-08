# Setting up Blocks Unreal environment for AirSim

Blocks environment comes with the repo and is designed to be lightweight in size. That means its very basic but fast.

Here are quick steps to get our build-in Blocks environment up and running:

1. Make sure you have [built the AirSim](build.md), have [setup your Pixhawk](prereq.md) and connected to USB port.
2. Navigate to folder `AirSim/Unreal/Environments/Blocks`.
3. Run `update_from_git.bat`. This will copy all compiled binaries to Unreal plugin folder.
4. Right click on `Blocks.uproject` and click `Generate Visual Studio Project files`.
5. Double click on generated .sln file to open in Visual Studio 2015 Update 3.
6. Make sure `Blocks` project is the startup project and press F5 to run.
7. You might get some errors/warnings like "_BuitData" file is missing which you can ignore. Press the Play button in Unreal Editor and you will see something like below.

[![Blocks Demo Video](images/blocks_video.png)](https://www.youtube.com/watch?v=-r_QGaxMT4A)