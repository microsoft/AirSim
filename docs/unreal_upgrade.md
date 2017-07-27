
# Upgrading Unreal Engine Version

## Blocks Project

If you are using Blocks project that comes with AirSim then you don't need to do anything other than [installling Unreal 4.16](build.md).

## Your Own Unreal Project
If you have your own Unreal project using AirSim plugin then you need to upgrade your project to use Unreal 4.16. 

### Option 1: Just Recreate Project

If your project doesn't have any code or assets other than environment you downloaded then you can also simply [recreate the project in Unreal 4.16 Editor](unreal_custenv.md) and then copy Plugins folder from `AirSim/Unreal/Plugins`. 

### Option 2: Modify Few Files

Unreal 4.16 Build system has breaking changes. So you need to modify your *.Build.cs and *.Target.cs which you can find in `Source` folder of your Unreal project. So what are those changes? Below is the gist of it but you should really refer to [Unreal's official 4.16 transition post](https://forums.unrealengine.com/showthread.php?145757-C-4-16-Transition-Guide).

#### In your project's *.Target.cs
1. Change the contructor from, `public MyProjectTarget(TargetInfo Target)` to `public MyProjectTarget(TargetInfo Target) : base(Target)`

2. Remove `SetupBinaries` method if you have one and instead add following line in contructor above: `ExtraModuleNames.AddRange(new string[] { "MyProject" });`

#### In your project's *.Build.cs
Change the constructor from `public MyProject(TargetInfo Target)` to `public MyProject(ReadOnlyTargetRules Target) : base(Target)`.

#### In your *.uproject
Remove line for `EngineAssociation`

#### And finally...
1. Make sure [Unreal 4.16 is installed](build,md).
2. Double click on your project's `*.uproject` file.
3. If you are asked to select Unreal version, select 4.16.
4. The warning box might show only "Open Copy" button. Don't click that. Instead click on More Options which will reveal more buttons. Choose Convert-In-Place option. Causion: Always keep backup of your project first!
5. If you don't have anything nasty, in place conversion should go through and you are now on new version of Unreal!
