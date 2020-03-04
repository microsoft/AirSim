@echo off
set SCRIPT_PATH=%~dp0
cd %~dp0
if "%1" == "" goto :usage
msbuild MavLinkComGenerator.csproj

bin\Debug\MavLinkComGenerator.exe -xml:%1 -out:%SCRIPT_PATH%\..\include
copy ..\include\MavLinkMessages.cpp ..\src
del ..\include\MavLinkMessages.cpp

goto :eof

:usage
echo we need the location of the mavlink 2.0 "common.xml" as input argument
