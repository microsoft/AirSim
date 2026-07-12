@echo off
setlocal
call "D:\VS2022\Community\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64
if ERRORLEVEL 1 (
  echo Failed to initialize VS DevCmd. >&2
  exit /b 1
)
cd /d E:\Project\AI\python\project\AirSimMy

set MSB=msbuild -maxcpucount:12 /p:Platform=x64 /p:Configuration=Debug /p:TrackFileAccess=false

%MSB% HelloDrone\HelloDrone.vcxproj || goto :fail
%MSB% HelloCar\HelloCar.vcxproj || goto :fail
%MSB% DroneServer\DroneServer.vcxproj || goto :fail
%MSB% DroneShell\DroneShell.vcxproj || goto :fail
%MSB% Examples\Examples.vcxproj || goto :fail
%MSB% HelloSpawnedDrones\HelloSpawnedDrones.vcxproj || goto :fail

echo All Debug sample projects built successfully.
exit /b 0

:fail
echo Sample Debug build failed. See errors above.
exit /b 1

