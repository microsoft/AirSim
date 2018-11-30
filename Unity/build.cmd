REM //---------- copy binaries and include for MavLinkCom in deps ----------
msbuild AirLibWrapper\AirsimWrapper.sln  /target:Clean /target:Build  /property:Configuration=Release /property:Platform=x64
if ERRORLEVEL 1 goto :buildfailed
copy /Y AirLibWrapper\x64\Release\AirsimWrapper.dll  UnityDemo\Assets\Plugins

REM //---------- done building ----------
exit /b 0

:buildfailed
echo(
echo #### Build failed - see messages above. 1>&2
exit /b 1