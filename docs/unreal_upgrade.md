# Upgrading to Unreal Engine 4.24

These instructions apply if you are already using AirSim on Unreal Engine 4.16. If you have never installed AirSim, please see [How to get it](https://github.com/microsoft/airsim#how-to-get-it).

**Caution:** The below steps will delete any of your unsaved work in AirSim or Unreal folder.

## Do this first

### For Windows Users
1. Install Visual Studio 2019 with VC++, Python and C#.
2. Install UE 4.24 through Epic Games Launcher.
3. Start `x64 Native Tools Command Prompt for VS 2019` and navigate to AirSim repo.
4. Run `clean_rebuild.bat` to remove all unchecked/extra stuff and rebuild everything.

### For Linux Users
1. From your AirSim repo folder, run 'clean_rebuild.sh`.
2. Rename or delete your existing folder for Unreal Engine.
3. Follow step 1 and 2 to [install Unreal Engine 4.24](https://github.com/Microsoft/AirSim/blob/master/docs/build_linux.md#install-and-build).

## Upgrading Your Custom Unreal Project
If you have your own Unreal project created in an older version of Unreal Engine then you need to upgrade your project to Unreal 4.24. To do this,

1. Open .uproject file and look for the line `"EngineAssociation"` and make sure it reads like `"EngineAssociation": "4.24"`.
2. Delete `Plugins/AirSim` folder in your Unreal project's folder.
3. Go to your AirSim repo folder and copy `Unreal\Plugins` folder to your Unreal project's folder.
4. Copy *.bat (or *.sh for Linux) from `Unreal\Environments\Blocks` to your project's folder.
5. Run `clean.bat` (or `clean.sh` for Linux) followed by `GenerateProjectFiles.bat` (only for Windows).

## FAQ

### How to force Unreal to build Solution for Visual Studio 2019?

If the default `update_from_git.bat` file results in VS 2017 project, then you may need to run the `C:\Program Files\Epic Games\UE_4.24\Engine\Binaries\DotNET\UnrealBuildTool.exe` tool manually, with the command line options `-projectfiles -project=<your.uproject>  -game -rocket -progress -2019`.

If you are upgrading from 4.18 to 4.24 you may also need to add `BuildSettingsVersion.V2` to your `*.Target.cs` and `*Editor.Target.cs` build files, like this:

```c#
	public AirSimNHTestTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.V2;
		ExtraModuleNames.AddRange(new string[] { "AirSimNHTest" });
	}
```

### I have an Unreal project that is older than 4.16. How do I upgrade it?

#### Option 1: Just Recreate Project
If your project doesn't have any code or assets other than environment you downloaded then you can also simply [recreate the project in Unreal 4.24 Editor](unreal_custenv.md) and then copy Plugins folder from `AirSim/Unreal/Plugins`.

#### Option 2: Modify Few Files
Unreal versions newer than Unreal 4.15 has breaking changes. So you need to modify your *.Build.cs and *.Target.cs which you can find in the `Source` folder of your Unreal project. So what are those changes? Below is the gist of it but you should really refer to [Unreal's official 4.16 transition post](https://forums.unrealengine.com/showthread.php?145757-C-4-16-Transition-Guide).

##### In your project's *.Target.cs
1. Change the contructor from, `public MyProjectTarget(TargetInfo Target)` to `public MyProjectTarget(TargetInfo Target) : base(Target)`

2. Remove `SetupBinaries` method if you have one and instead add following line in contructor above: `ExtraModuleNames.AddRange(new string[] { "MyProject" });`

##### In your project's *.Build.cs
Change the constructor from `public MyProject(TargetInfo Target)` to `public MyProject(ReadOnlyTargetRules Target) : base(Target)`.

##### And finally...
Follow above steps to continue the upgrade. The warning box might show only "Open Copy" button. Don't click that. Instead, click on More Options which will reveal more buttons. Choose `Convert-In-Place option`. *Caution:* Always keep backup of your project first! If you don't have anything nasty, in place conversion should go through and you are now on the new version of Unreal.
