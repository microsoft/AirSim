cd /d "D:\git\theseus\catkin_ws\src\AirSim\MavLinkCom" &msbuild "MavLinkCom.vcxproj" /t:sdvViewer /p:configuration="Debug" /p:platform=x64
exit %errorlevel% 