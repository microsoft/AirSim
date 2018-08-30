mkdir Plugins
mklink /D Plugins\AirSim ..\..\..\Plugins\AirSim
mklink /D Plugins\AirSim\Source\AirLib ..\..\..\..\AirLib
cmd /c clean.bat
cmd /c GenerateProjectFiles.bat

pause
