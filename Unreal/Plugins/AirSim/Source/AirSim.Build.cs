// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

using UnrealBuildTool;
using System.IO;

public class AirSim : ModuleRules
{
    const string readmurl = "https://github.com/Microsoft/AirSim/blob/master/docs/install_eigen.md";

    private string ModulePath
    {
        get { return ModuleDirectory; }
    }

    private string AirLibPath
    {
        get { return Path.GetFullPath(Path.Combine(ModulePath, "AirLib")); }
    }
    private string AirSimPluginPath
    {
        get { return Path.GetFullPath(Directory.GetParent(ModulePath).ToString()); }
    }
    private string ProjectBinariesPath
    {
        get {
            return Path.GetFullPath(Path.Combine(
                Directory.GetParent(ModulePath).Parent.Parent.ToString(), "Binaries"));
        }
    }

    private enum CompileMode
    {
        HeaderOnlyNoRpc,
        HeaderOnlyWithRpc,
        CppCompileNoRpc,
        CppCompileWithRpc
    }

    private void SetupCompileMode(CompileMode mode, ReadOnlyTargetRules Target)
    {
        LoadAirSimDependency(Target, "MavLinkCom", "MavLinkCom");

        switch (mode)
        {
            case CompileMode.HeaderOnlyNoRpc:
                Definitions.Add("AIRLIB_HEADER_ONLY=1");
                Definitions.Add("AIRLIB_NO_RPC=1");
                AddLibDependency("AirLib", Path.Combine(AirLibPath, "lib"), "AirLib", Target, false);
                break;
            case CompileMode.HeaderOnlyWithRpc:
                Definitions.Add("AIRLIB_HEADER_ONLY=1");
                AddLibDependency("AirLib", Path.Combine(AirLibPath, "lib"), "AirLib", Target, false);
                LoadAirSimDependency(Target, "rpclib", "rpc");
                break;
            case CompileMode.CppCompileNoRpc:
                LoadAirSimDependency(Target, "MavLinkCom", "MavLinkCom");
                Definitions.Add("AIRLIB_NO_RPC=1");
                break;
            case CompileMode.CppCompileWithRpc:
                LoadAirSimDependency(Target, "rpclib", "rpc");
                break;
            default:
                throw new System.Exception("CompileMode specified in plugin's Build.cs file is not recognized");
        }

    }

    public AirSim(ReadOnlyTargetRules Target) : base(Target)
    {
        //bEnforceIWYU = true; //to support 4.16
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        //below is no longer supported in 4.16
        bEnableExceptions = true;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "ImageWrapper", "RenderCore", "RHI" });
        PrivateDependencyModuleNames.AddRange(new string[] { "UMG", "Slate", "SlateCore" });

        //suppress VC++ proprietary warnings
        Definitions.Add("_SCL_SECURE_NO_WARNINGS=1");
        Definitions.Add("CRT_SECURE_NO_WARNINGS=1");

        PrivateIncludePaths.Add(Path.Combine(AirLibPath, "include"));
        PrivateIncludePaths.Add(Path.Combine(AirLibPath, "deps", "eigen3"));
        AddOSLibDependencies(Target);

        SetupCompileMode(CompileMode.CppCompileWithRpc, Target);
    }

    private void AddOSLibDependencies(ReadOnlyTargetRules Target)
    {
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            // for SHGetFolderPath.
            PublicAdditionalLibraries.Add("Shell32.lib");

            // XInput for JoyStick, make sure to delay load this because we use generated DLL from x360ce
            PublicDelayLoadDLLs.Add("Xinput9_1_0.dll");
            //Lib for the xinput DLL
            //this should be in path, typically at C:\Program Files (x86)\Windows Kits\8.1\Lib\winv6.3\um\x64
            //typically gets installed with Visual Studio or DirectX
            PublicAdditionalLibraries.Add("Xinput9_1_0.lib");

            RuntimeDependencies.Add(new RuntimeDependency(Path.Combine(ProjectBinariesPath, "Win64", "xinput1_3.dll")));
            RuntimeDependencies.Add(new RuntimeDependency(Path.Combine(ProjectBinariesPath, "Win64", "x360ce.ini")));
            System.Console.WriteLine(Directory.GetParent( ModulePath));
            System.Console.WriteLine(Directory.GetParent(ModulePath).Parent);
            System.Console.WriteLine(Directory.GetParent(ModulePath).Parent.Parent);

            System.Console.WriteLine(ProjectBinariesPath);
        }
    }

    private bool LoadAirSimDependency(ReadOnlyTargetRules Target, string LibName, string LibFileName)
    {
        string LibrariesPath = Path.Combine(AirLibPath, "deps", LibName, "lib");
        return AddLibDependency(LibName, LibrariesPath, LibFileName, Target, true);
    }

    private bool AddLibDependency(string LibName, string LibPath, string LibFileName, ReadOnlyTargetRules Target, bool IsAddLibInclude)
    {
        string PlatformString = (Target.Platform == UnrealTargetPlatform.Win64 || Target.Platform == UnrealTargetPlatform.Mac) ? "x64" : "x86";
        string ConfigurationString = (Target.Configuration == UnrealTargetConfiguration.Debug) ? "Debug" : "Release";
        bool isLibrarySupported = false;


        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            isLibrarySupported = true;

            PublicAdditionalLibraries.Add(Path.Combine(LibPath, PlatformString, ConfigurationString, LibFileName + ".lib"));
        } else if (Target.Platform == UnrealTargetPlatform.Linux || Target.Platform == UnrealTargetPlatform.Mac) {
            isLibrarySupported = true;
            PublicAdditionalLibraries.Add(Path.Combine(LibPath, "lib" + LibFileName + ".a"));
        }

        if (isLibrarySupported && IsAddLibInclude)
        {
            // Include path
            PrivateIncludePaths.Add(Path.Combine(AirLibPath, "deps", LibName, "include"));
        }

        Definitions.Add(string.Format("WITH_" + LibName.ToUpper() + "_BINDING={0}", isLibrarySupported ? 1 : 0));

        return isLibrarySupported;
    }
}
