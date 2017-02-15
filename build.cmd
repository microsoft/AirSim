msbuild /p:Platform=x64 /p:Configuration=Debug DroneShell.sln
msbuild /p:Platform=x64 /p:Configuration=Release DroneShell.sln

set MAVLINK_TARGET_LIB=AirLib\deps\MavLinkCom\lib
if NOT exist %MAVLINK_TARGET_LIB% mkdir %MAVLINK_TARGET_LIB%

if NOT exist Unreal\Plugins\AirSim\Source\AirLib mkdir Unreal\Plugins\AirSim\Source\AirLib

robocopy /MIR MavLinkCom\lib AirLib\deps\MavLinkCom\lib
robocopy /MIR AirLib Unreal\Plugins\AirSim\Source\AirLib  /XD temp