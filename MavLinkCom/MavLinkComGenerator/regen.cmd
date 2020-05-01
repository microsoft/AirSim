@echo off
set SCRIPT_PATH=%~dp0
cd %~dp0
msbuild MavLinkComGenerator.csproj

bin\Debug\MavLinkComGenerator.exe -xml:%SCRIPT_PATH%\..\mavlink\message_definitions\common.xml -out:%SCRIPT_PATH%\..\include
copy ..\include\MavLinkMessages.cpp ..\src
del ..\include\MavLinkMessages.cpp

goto :eof
