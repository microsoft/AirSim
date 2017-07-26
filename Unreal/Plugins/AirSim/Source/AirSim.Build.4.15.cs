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

    private string AirSimPath
    {
        get { return Path.GetFullPath(Path.Combine(ModulePath, "AirLib")); }
    }

    private enum CompileMode
    {
        HeaderOnlyNoRpc,
        HeaderOnlyWithRpc,
        CppCompileNoRpc,
        CppCompileWithRpc
    }

    private void SetupCompileMode(CompileMode mode, TargetInfo Target)
    {
        LoadAirSimDependency(Target, "MavLinkCom", "MavLinkCom");

        switch (mode)
        {
            case CompileMode.HeaderOnlyNoRpc:
                Definitions.Add("AIRLIB_HEADER_ONLY=1");
                Definitions.Add("AIRLIB_NO_RPC=1");
                AddLibDependency("AirLib", Path.Combine(AirSimPath, "lib"), "AirLib", Target, false);
                break;
            case CompileMode.HeaderOnlyWithRpc:
                Definitions.Add("AIRLIB_HEADER_ONLY=1");
                AddLibDependency("AirLib", Path.Combine(AirSimPath, "lib"), "AirLib", Target, false);
                LoadAirSimDependency(Target, "rpclib", "rpc");
                break;
            case CompileMode.CppCompileNoRpc:
                LoadAirSimDependency(Target, "MavLinkCom", "MavLinkCom");
                Definitions.Add("AIRLIB_NO_RPC=1");
                break;
            case CompileMode.CppCompileWithRpc:
                LoadAirSimDependency(Target, "MavLinkCom", "MavLinkCom");
                LoadAirSimDependency(Target, "rpclib", "rpc");
                break;
            default:
                throw new System.Exception("CompileMode specified in plugin's Build.cs file is not recognized");
        }

    }

    public AirSim(TargetInfo Target)
    {
        bEnforceIWYU = false;
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        bEnableExceptions = true;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "RenderCore", "RHI" });
        PrivateDependencyModuleNames.AddRange(new string[] { "UMG", "Slate", "SlateCore" });

        //suppress VC++ proprietary warnings
        Definitions.Add("_SCL_SECURE_NO_WARNINGS=1");
        Definitions.Add("CRT_SECURE_NO_WARNINGS=1");

        PrivateIncludePaths.Add(Path.Combine(AirSimPath, "include"));
        PrivateIncludePaths.Add(Path.Combine(AirSimPath, "deps", "eigen3"));
        AddOSLibDependencies(Target);

        SetupCompileMode(CompileMode.HeaderOnlyWithRpc, Target);
    }

    private void AddOSLibDependencies(TargetInfo Target)
    {
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            // for SHGetFolderPath.
            PublicAdditionalLibraries.Add("Shell32.lib");

            // XInput for JoyStick, make sure to delay load this because we use generated DLL from x360ce
            PublicDelayLoadDLLs.Add("xinput9_1_0.dll");
            //Lib for the xinput DLL
            //this should be in path, typically at C:\Program Files (x86)\Windows Kits\8.1\Lib\winv6.3\um\x64
            //typically gets installed with Visual Studio
            PublicAdditionalLibraries.Add("xinput9_1_0.lib");
        }
    }

    private bool LoadAirSimDependency(TargetInfo Target, string LibName, string LibFileName)
    {
        string LibrariesPath = Path.Combine(AirSimPath, "deps", LibName, "lib");
        return AddLibDependency(LibName, LibrariesPath, LibFileName, Target, true);
    }

    private bool AddLibDependency(string LibName, string LibPath, string LibFileName, TargetInfo Target, bool IsAddLibInclude)
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
            PrivateIncludePaths.Add(Path.Combine(AirSimPath, "deps", LibName, "include"));
        }

        Definitions.Add(string.Format("WITH_" + LibName.ToUpper() + "_BINDING={0}", isLibrarySupported ? 1 : 0));

        return isLibrarySupported;
    }
}
