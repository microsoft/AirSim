msbuild /p:Platform=x64 /p:Configuration=Debug AirSim.sln
msbuild /p:Platform=x64 /p:Configuration=Release AirSim.sln

set MAVLINK_TARGET_LIB=AirLib\deps\MavLinkCom\lib
if NOT exist %MAVLINK_TARGET_LIB% mkdir %MAVLINK_TARGET_LIB%

set MAVLINK_TARGET_INCLUDE=AirLib\deps\MavLinkCom\include
if NOT exist %MAVLINK_TARGET_INCLUDE% mkdir %MAVLINK_TARGET_INCLUDE%

if NOT exist Unreal\Plugins\AirSim\Source\AirLib mkdir Unreal\Plugins\AirSim\Source\AirLib

robocopy /MIR MavLinkCom\include %MAVLINK_TARGET_INCLUDE%
robocopy /MIR MavLinkCom\lib %MAVLINK_TARGET_LIB%
robocopy /MIR AirLib Unreal\Plugins\AirSim\Source\AirLib  /XD temp