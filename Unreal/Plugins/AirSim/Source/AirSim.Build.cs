using UnrealBuildTool;
using System.IO;

public class AirSim : ModuleRules
{
    const string readmurl = "https://github.com/Microsoft/AirSim/blob/master/docs/install_boost.md";

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
        UEBuildConfiguration.bForceEnableExceptions = true;
        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "RenderCore" });
        PrivateDependencyModuleNames.AddRange(new string[] { "UMG", "Slate", "SlateCore" });

        //suppress VC++ proprietary warnings
        Definitions.Add("_SCL_SECURE_NO_WARNINGS=1");
        Definitions.Add("CRT_SECURE_NO_WARNINGS=1");

        AddEigenDependency();
        PrivateIncludePaths.Add(Path.Combine(AirSimPath, "include"));
        AddOSLibDependencies(Target);
        AddBoostDependency();
        LoadAirSimDependency(Target, "MavLinkCom", "MavLinkCom");

        SetupCompileMode(CompileMode.HeaderOnlyWithRpc, Target);
    }

    private void AddEigenDependency()
    {
        string eigenPath = System.Environment.GetEnvironmentVariable("EIGEN_ROOT");
        if (string.IsNullOrEmpty(eigenPath) || !System.IO.Directory.Exists(eigenPath) || !System.IO.Directory.Exists(Path.Combine(eigenPath, "eigen3")))
        {
            throw new System.Exception("EIGEN_ROOT is not defined, or points to a non-existant directory, please set this environment variable.  " +
                "See readme: " + readmurl);
        }
        PrivateIncludePaths.Add(Path.Combine(eigenPath, "eigen3"));
    }

    private void AddOSLibDependencies(TargetInfo Target)
    {
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            // for SHGetFolderPath.
            PublicAdditionalLibraries.Add("Shell32.lib");
        }
    }

    private void AddBoostDependency()
    {
        string boost = System.Environment.GetEnvironmentVariable("BOOST_ROOT");
        if (string.IsNullOrEmpty(boost) || !System.IO.Directory.Exists(boost))
        {
            throw new System.Exception("BOOST_ROOT is not defined, or points to a non-existant directory, please set this environment variable.  " +
                "See: " + readmurl);
        }
        string lib = Path.Combine(boost, "stage", "lib");
        if (!System.IO.Directory.Exists(lib))
        {
            throw new System.Exception("Please build boost and make sure the libraries are at " + lib + ". " +
                "See: " + readmurl);
        }

        bool found = System.IO.Directory.GetFiles(lib, "libboost_system-*.lib").Length > 0;
        if (!found)
        {
            throw new System.Exception("Not finding libboost_system-*.lib in " + lib + ".  " +
                "See: " + readmurl);
        }

        PublicLibraryPaths.Add(Path.Combine(lib));
    }


    private bool LoadAirSimDependency(TargetInfo Target, string LibName, string LibFileName)
    {
        string LibrariesPath = Path.Combine(AirSimPath, "deps", LibName, "lib");
        return AddLibDependency(LibName, LibrariesPath, LibFileName, Target, true);
    }

    private bool AddLibDependency(string LibName, string LibPath, string LibFileName, TargetInfo Target, bool IsAddLibInclude)
    {
        string PlatformString = (Target.Platform == UnrealTargetPlatform.Win64) ? "x64" : "x86";
        string ConfigurationString = (Target.Configuration == UnrealTargetConfiguration.Debug) ? "Debug" : "Release";
        bool isLibrarySupported = false;


        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            isLibrarySupported = true;

            PublicAdditionalLibraries.Add(Path.Combine(LibPath, PlatformString, ConfigurationString, LibFileName + ".lib"));
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
