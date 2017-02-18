
msbuild /p:Platform=x64 /p:Configuration=Debug AirSim.sln /t:Clean
if ERRORLEVEL 1 goto :buildfailed
msbuild /p:Platform=x64 /p:Configuration=Release AirSim.sln /t:Clean
if ERRORLEVEL 1 goto :buildfailed
goto :eof

:buildfailed
echo #### Build failed
goto :eof