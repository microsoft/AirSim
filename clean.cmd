
msbuild /p:Platform=x64 /p:Configuration=Debug DroneShell.sln /t:Clean
if ERRORLEVEL 1 goto :buildfailed
msbuild /p:Platform=x64 /p:Configuration=Release DroneShell.sln /t:Clean
if ERRORLEVEL 1 goto :buildfailed
goto :eof

:buildfailed
echo #### Build failed
goto :eof