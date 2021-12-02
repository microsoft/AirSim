# Development Workflow

Below is the guide on how to perform different development activities while working with AirSim. If you are new to Unreal Engine based projects and want to contribute to AirSim or make your own forks for your custom requirements, this might save you some time.

## Development Environment
### OS
We highly recommend Windows 10 and Visual Studio 2019 as your development environment. The support for other OSes and IDE is unfortunately not as mature on the Unreal Engine side and you may risk severe loss of productivity trying to do workarounds and jumping through the hoops.

### Hardware
We recommend GPUs such as NVidia 1080 or NVidia Titan series with powerful desktop such as one with 64GB RAM, 6+ cores, SSDs and 2-3 displays (ideally 4K). We have found HP Z840 work quite well for our needs. The development experience on high-end laptops is generally sub-par compared to powerful desktops however they might be useful in a pinch. You generally want laptops with discrete NVidia GPU (at least M2000 or better) with 64GB RAM, SSDs and hopefully 4K display. We have found models such as Lenovo P50 work well for our needs. Laptops with only integrated graphics might not work well.

## Updating and Changing AirSim Code

### Overview
AirSim is designed as plugin. This means it can't run by itself, you need to put it in an Unreal project (we call it "environment"). So building and testing AirSim has two steps: (1) build the plugin (2) deploy plugin in Unreal project and run the project. 

The first step is accomplished by build.cmd available in AirSim root. This command will update everything you need for the plugin in the `Unreal\Plugins` folder. So to deploy the plugin, you just need to copy `Unreal\Plugins` folder in to your Unreal project folder. Next you should remove all  intermediate files in your Unreal project and then regenerate .sln file for your Unreal project. To do this, we have two handy .bat files in `Unreal\Environments\Blocks` folder: `clean.bat` and `GenerateProjectFiles.bat`. So just run these bat files in sequence from root of your Unreal project. Now you are ready to open new .sln in Visual Studio and press F5 to run it.

### Steps
Below are the steps we use to make changes in AirSim and test them out. The best way to do development in AirSim code is to use [Blocks project](unreal_blocks.md). This is the light weight project so compile time is relatively faster. Generally the workflow is,

```
REM //Use x64 Native Tools Command Prompt for VS 2019
REM //Navigate to AirSim repo folder

git pull                          
build.cmd                        
cd Unreal\Environments\Blocks         
update_from_git.bat
start Blocks.sln
```

Above commands first builds the AirSim plugin and then deploys it to Blocks project using handy `update_from_git.bat`. Now you can work inside Visual Studio solution, make changes to the code and just run F5 to build, run and test your changes. The debugging, break points etc should work as usual. 

After you are done with you code changes, you might want to push your changes back to AirSim repo or your own fork or you may deploy the new plugin to your custom Unreal project. To do this, go back to command prompt and first update the AirSim repo folder:


```
REM //Use x64 Native Tools Command Prompt for VS 2019
REM //run this from Unreal\Environments\Blocks 

update_to_git.bat
build.cmd
```

Above command will transfer your code changes from Unreal project folder back to `Unreal\Plugins` folder. Now your changes are ready to be pushed to AirSim repo or your own fork. You can also copy `Unreal\Plugins` to your custom Unreal engine project and see if everything works in your custom project as well.

### Take Away
 Once you understand how Unreal Build system and plugin model works as well as why we are doing above steps, you should feel quite comfortable in following this workflow. Don't be afraid of opening up .bat files to peek inside and see what its doing. They are quite minimal and straightforward (except, of course, build.cmd - don't look in to that one).

## FAQ

#### I made changes in code in Blocks project but its not working.
When you press F5 or F6 in Visual Studio to start build, the Unreal Build system kicks in and it tries to find out if any files are dirty and what it needs to build. Unfortunately, it often fails to recognize dirty files that is not the code that uses Unreal headers and object hierarchy. So, the trick is to just make some file dirty that Unreal Build system always recognizes. My favorite one is AirSimGameMode.cpp. Just insert a line, delete it and save the file.

#### I made changes in the code outside of Visual Studio but its not working.
Don't do that! Unreal Build system *assumes* that you are using Visual Studio and it does bunch of things to integrate with Visual Studio. If you do insist on using other editors then look up how to do command line builds in Unreal projects OR see docs on your editor on how it can integrate with Unreal build system OR run `clean.bat` + `GenerateProjectFiles.bat` to make sure VS solution is in sync.

#### I'm trying to add new file in the Unreal Project and its not working.
It won't! While you are indeed using Visual Studio solution, remember that this solution was actually generated by Unreal Build system. If you want to add new files in your project, first shut down Visual Studio, add an empty file at desired location and then run `GenerateProjectFiles.bat` which will scan all files in your project and then re-create the .sln file. Now open this new .sln file and you are in business.

#### I copied Unreal\Plugins folder but nothing happens in Unreal Project.
First make sure your project's .uproject file is referencing the plugin. Then make sure you have run `clean.bat` and then `GenerateProjectFiles.bat` as described in Overview above.

#### I have multiple Unreal projects with AirSim plugin. How do I update them easily?
You are in luck! We have `build_all_ue_projects.bat` which exactly does that. Don't treat it as black box (at least not yet), open it up and see what it does.  It has 4 variables that are being set from command line args. If these args is not supplied they are set to default values in next set of statements. You might want to change default values for the paths. This batch file builds AirSim plugin, deploys it to all listed projects (see CALL statements later in the batch file), runs packaging for those projects and puts final binaries in specified folder - all in one step! This is what we use to create our own binary releases.

#### How do I contribute back to AirSim?
Before making any changes make sure you have created your feature branch. After you test your code changes in Blocks environment, follow the [usual steps](https://akrabat.com/the-beginners-guide-to-contributing-to-a-github-project/) to make contributions just like any other GitHub projects. Please use rebase and squash merge, for more information see [An introduction to Git merge and rebase: what they are, and how to use them](https://www.freecodecamp.org/news/an-introduction-to-git-merge-and-rebase-what-they-are-and-how-to-use-them-131b863785f/).



