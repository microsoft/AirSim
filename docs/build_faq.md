# FAQ

---

## Windows build

* [How to force Unreal to use Visual Studio 2019?](#how-to-force-unreal-to-use-visual-studio-2019)
* [I get error: 'where' is not recognized as an internal or external command](#i-get-error-where-is-not-recognized-as-an-internal-or-external-command)
* [I'm getting error `<MyProject> could not be compiled. Try rebuilding from source manually`](#im-getting-error-myproject-could-not-be-compiled-try-rebuilding-from-source-manually)
* [I get `error C100 : An internal error has occurred in the compiler` when running build.cmd](#i-get-error-c100--an-internal-error-has-occurred-in-the-compiler-when-running-buildcmd)
* [I get error "'corecrt.h': No such file or directory" or "Windows SDK version 8.1 not found"](#i-get-error-corecrth-no-such-file-or-directory-or-windows-sdk-version-81-not-found)
* [How do I use PX4 firmware with AirSim?](#how-do-i-use-px4-firmware-with-airsim)
* [I made changes in Visual Studio but there is no effect](#i-made-changes-in-visual-studio-but-there-is-no-effect)
* [Unreal still uses VS2015 or I'm getting some link error](#unreal-still-uses-vs2015-or-im-getting-some-link-error)

---

## Linux build

* [I'm getting error `<MyProject> could not be compiled. Try rebuilding from source manually`.](#im-getting-error-myproject-could-not-be-compiled-try-rebuilding-from-source-manually)
* [Unreal crashed! How do I know what went wrong?](#unreal-crashed-how-do-i-know-what-went-wrong)
* [How do I use an IDE on Linux?](#how-do-i-use-an-ide-on-linux)
* [Can I cross compile for Linux from a Windows machine?](#can-i-cross-compile-for-linux-from-a-windows-machine)
* [What compiler and stdlib does AirSim use?](#what-compiler-and-stdlib-does-airsim-use)
* [What version of CMake does the AirSim build use?](#what-version-of-cmake-does-the-airsim-build-use)
* [Can I compile AirSim in BashOnWindows?](#can-i-compile-airsim-in-bashonwindows)
* [Where can I find more info on running Unreal on Linux?](#where-can-i-find-more-info-on-running-unreal-on-linux)

---

## Other

* [Packaging AirSim](#packaging-a-binary-including-the-airsim-plugin)

---

<!-- ======================================================================= -->
## Windows build
<!-- ======================================================================= -->

###### How to force Unreal to use Visual Studio 2019?

>If the default `update_from_git.bat` file results in VS 2017 project, then you may need to run the `C:\Program Files\Epic Games\UE_4.25\Engine\Binaries\DotNET\UnrealBuildTool.exe` tool manually, with the command line options `-projectfiles -project=<your.uproject>  -game -rocket -progress -2019`.
>
>If you are upgrading from 4.18 to 4.25 you may also need to add `BuildSettingsVersion.V2` to your `*.Target.cs` and `*Editor.Target.cs` build files, like this:
>
>```c#
>	public AirSimNHTestTarget(TargetInfo Target) : base(Target)
>	{
>		Type = TargetType.Game;
>		DefaultBuildSettings = BuildSettingsVersion.V2;
>		ExtraModuleNames.AddRange(new string[] { "AirSimNHTest" });
>	}
>```
>
>You may also need to edit this file:
>
>```
>"%APPDATA%\Unreal Engine\UnrealBuildTool\BuildConfiguration.xml
>```
>
>And add this Compiler version setting:
>
>```xml
><Configuration xmlns="https://www.unrealengine.com/BuildConfiguration">
>  <WindowsPlatform>
>    <Compiler>VisualStudio2019</Compiler>
>  </WindowsPlatform>
></Configuration>
>```

<!-- ======================================================================= -->

###### I get error: 'where' is not recognized as an internal or external command

>You have to add `C:\WINDOWS\SYSTEM32` to your PATH enviroment variable.

<!-- ======================================================================= -->

###### I'm getting error `<MyProject> could not be compiled. Try rebuilding from source manually`

>This will occur when there are compilation errors. Logs are stored in `<My-Project>\Saved\Logs` which can be used to figure out the problem.
>
>A common problem could be Visual Studio version conflict, AirSim uses VS 2019 while UE is using VS 2017, this can be found by searching for `2017` in the Log file. In that case, see the answer above.
>
>If you have modified the AirSim plugin files, then you can right-click the `.uproject` file, select `Generate Visual Studio solution file` and then open the `.sln` file in VS to fix the errors and build again.

<!-- ======================================================================= -->

###### I get `error C100 : An internal error has occurred in the compiler` when running build.cmd

>We have noticed this happening with VS version `15.9.0` and have checked-in a workaround in AirSim code. If you have this VS version, please make sure to pull the latest AirSim code.

<!-- ======================================================================= -->

###### I get error "'corecrt.h': No such file or directory" or "Windows SDK version 8.1 not found"

>Very likely you don't have [Windows SDK](https://developercommunity.visualstudio.com/content/problem/3754/cant-compile-c-program-because-of-sdk-81cant-add-a.html) installed with Visual Studio.

<!-- ======================================================================= -->

###### How do I use PX4 firmware with AirSim?

>By default, AirSim uses its own built-in firmware called [simple_flight](simple_flight.md). There is no additional setup if you just want to go with it. If you want to switch to using PX4 instead then please see [this guide](px4_setup.md).

<!-- ======================================================================= -->

###### I made changes in Visual Studio but there is no effect

>Sometimes the Unreal + VS build system doesn't recompile if you make changes to only header files. To ensure a recompile, make some Unreal based cpp file "dirty" like AirSimGameMode.cpp.

<!-- ======================================================================= -->

###### Unreal still uses VS2015 or I'm getting some link error

>Running several versions of VS can lead to issues when compiling UE projects. One problem that may arise is that UE will try to compile with an older version of VS which may or may not work. There are two settings in Unreal, one for for the engine and one for the project, to adjust the version of VS to be used.
>
>1. Edit -> Editor preferences -> General -> Source code -> Source Code Editor
>2. Edit -> Project Settings -> Platforms -> Windows -> Toolchain ->CompilerVersion
>
>In some cases, these settings will still not lead to the desired result and errors such as the following might be produced: LINK : fatal error LNK1181: cannot open input file 'ws2_32.lib'
>
>To resolve such issues the following procedure can be applied:
>
>1. Uninstall all old versions of VS using the [VisualStudioUninstaller](https://github.com/Microsoft/VisualStudioUninstaller/releases)
>2. Repair/Install VS 2019
>3. Restart machine and install Epic launcher and desired version of the engine

---

## Linux build
<!-- ======================================================================= -->

###### I'm getting error `<MyProject> could not be compiled. Try rebuilding from source manually`.

>This could either happen because of compile error or the fact that your gch files are outdated. Look in to your console window. Do you see something like below?
>
>`fatal error: file '/usr/include/linux/version.h''/usr/include/linux/version.h' has been modified since the precompiled header`
>
>If this is the case then look for *.gch file(s) that follows after that message, delete them and try again. Here's [relevant thread](https://answers.unrealengine.com/questions/412349/linux-ue4-build-precompiled-header-fatal-error.html) on Unreal Engine forums.
>
>If you see other compile errors in console then open up those source files and see if it is due to changes you made. If not, then report it as issue on GitHub.

<!-- ======================================================================= -->

###### Unreal crashed! How do I know what went wrong?

>Go to the `MyUnrealProject/Saved/Crashes` folder and search for the file `MyProject.log` within its subdirectories. At the end of this file you will see the stack trace and messages.
>You can also take a look at the `Diagnostics.txt` file.

<!-- ======================================================================= -->

###### How do I use an IDE on Linux?

>You can use Qt Creator or CodeLite. Instructions for Qt Creator are available [here](https://docs.unrealengine.com/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpQtCreator/index.html).

<!-- ======================================================================= -->

###### Can I cross compile for Linux from a Windows machine?

>Yes, you can, but we haven't tested it. You can find the instructions [here](https://docs.unrealengine.com/latest/INT/Platforms/Linux/GettingStarted/index.html).

<!-- ======================================================================= -->

###### What compiler and stdlib does AirSim use?

>We use the same compiler that Unreal Engine uses, **Clang 8**, and stdlib, **libc++**. AirSim's `setup.sh` will automatically download them.

<!-- ======================================================================= -->

###### What version of CMake does the AirSim build use?

>3.10.0 or higher. This is *not* the default in Ubuntu 16.04 so setup.sh installs it for you. You can check your CMake version using `cmake --version`. If you have an older version, follow [these instructions](cmake_linux.md) or see the [CMake website](https://cmake.org/install/).

<!-- ======================================================================= -->

###### Can I compile AirSim in BashOnWindows?

>Yes, however, you can't run Unreal from BashOnWindows. So this is kind of useful to check a Linux compile, but not for an end-to-end run.
>See the [BashOnWindows install guide](https://msdn.microsoft.com/en-us/commandline/wsl/install_guide).
>Make sure to have the latest version (Windows 10 Creators Edition) as previous versions had various issues.
>Also, don't invoke `bash` from `Visual Studio Command Prompt`, otherwise CMake might find VC++ and try and use that!

<!-- ======================================================================= -->

###### Where can I find more info on running Unreal on Linux?
>Start here: [Unreal on Linux](https://docs.unrealengine.com/latest/INT/Platforms/Linux/index.html)
>[Building Unreal on Linux](https://wiki.unrealengine.com/Building_On_Linux#Clang)
>[Unreal Linux Support](https://wiki.unrealengine.com/Linux_Support)
>[Unreal Cross Compilation](https://wiki.unrealengine.com/Compiling_For_Linux)

---

## Other
<!-- ======================================================================= -->

###### Packaging a binary including the AirSim plugin

>In order to package a custom environment with the AirSim plugin, there are a few project settings that are necessary for ensuring all required assets needed for AirSim are included inside the package. Under `Edit -> Project Settings... -> Project -> Packaging`, please ensure the following settings are configured properly:
>
>- `List of maps to include in a packaged build`: ensure one entry exists for `/AirSim/AirSimAssets` 
>- `Additional Asset Directories to Cook`: ensure one entry exists for `/AirSim/HUDAssets`
