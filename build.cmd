@echo off
REM //---------- set up variable ----------
setlocal
set ROOT_DIR=%~dp0

REM // Check command line arguments
set "noFullPolyCar="

if "%1"=="" goto noargs
if "%1"=="--no-full-poly-car" set "noFullPolyCar=y"

:noargs

REM //---------- make sure we have got all sub modules ----------
chdir /d %ROOT_DIR% 
ECHO Updating submodules...
git submodule update --init --recursive

REM //---------- if cmake doesn't exist then install it ----------
WHERE cmake >nul 2>nul
IF %ERRORLEVEL% NEQ 0 (
    call :installcmake
)

REM //---------- get High PolyCount SUV Car Model ------------
IF NOT EXIST Unreal\Plugins\AirSim\Content\VehicleAdv mkdir Unreal\Plugins\AirSim\Content\VehicleAdv
IF NOT EXIST Unreal\Plugins\AirSim\Content\VehicleAdv\SUV\v1.1.7 (
    IF NOT DEFINED noFullPolyCar (
        REM //leave some blank lines because powershell shows download banner at top of console
        ECHO(   
        ECHO(   
        ECHO(   
        ECHO *****************************************************************************************
        ECHO Downloading high-poly car assets.... The download is ~37MB and can take some time.
        ECHO To install without this assets, re-run build.cmd with the argument --no-full-poly-car
        ECHO *****************************************************************************************
       
        IF EXIST suv_download_tmp rmdir suv_download_tmp /q /s
        mkdir suv_download_tmp
        @echo on
        REM powershell -command "& { Start-BitsTransfer -Source https://github.com/Microsoft/AirSim/releases/download/v1.1.7/car_assets.zip -Destination suv_download_tmp\car_assets.zip }"
        REM powershell -command "& { (New-Object System.Net.WebClient).DownloadFile('https://github.com/Microsoft/AirSim/releases/download/v1.1.7/car_assets.zip', 'suv_download_tmp\car_assets.zip') }"
        powershell -command "& { iwr https://github.com/Microsoft/AirSim/releases/download/v1.1.7/car_assets.zip -OutFile suv_download_tmp\car_assets.zip }"
        @echo off
		rmdir /S /Q Unreal\Plugins\AirSim\Content\VehicleAdv\SUV
        powershell -command "& { Expand-Archive -Path suv_download_tmp\car_assets.zip -DestinationPath Unreal\Plugins\AirSim\Content\VehicleAdv }"
        rmdir suv_download_tmp /q /s
        
        REM //Don't fail the build if the high-poly car is unable to be downloaded
        REM //Instead, just notify users that the gokart will be used.
        IF NOT EXIST Unreal\Plugins\AirSim\Content\VehicleAdv\SUV ECHO Unable to download high-polycount SUV. Your AirSim build will use the default vehicle.
    ) else (
        ECHO Not downloading high-poly car asset. The default unreal vehicle will be used.
    )
)

REM //---------- get Eigen library ----------
IF NOT EXIST AirLib\deps mkdir AirLib\deps
IF NOT EXIST AirLib\deps\eigen3 (
    powershell -command "& { iwr http://bitbucket.org/eigen/eigen/get/3.3.2.zip -OutFile eigen3.zip }"
    powershell -command "& { Expand-Archive -Path eigen3.zip -DestinationPath AirLib\deps }"
    move AirLib\deps\eigen* AirLib\deps\del_eigen
    mkdir AirLib\deps\eigen3
    move AirLib\deps\del_eigen\Eigen AirLib\deps\eigen3\Eigen
    rmdir /S /Q AirLib\deps\del_eigen
    del eigen3.zip
)
IF NOT EXIST AirLib\deps\eigen3 goto :buildfailed

ECHO Starting cmake...
REM //---------- compile rpclib that we got from git submodule ----------
IF NOT EXIST external\rpclib\build mkdir external\rpclib\build
cd external\rpclib\build
REM cmake -G"Visual Studio 15 2017 Win64" ..
cmake -G"Visual Studio 14 2015 Win64" ..
cmake --build .
cmake --build . --config Release
if ERRORLEVEL 1 goto :buildfailed
chdir /d %ROOT_DIR% 

REM //---------- copy rpclib binaries and include folder inside AirLib folder ----------
set RPCLIB_TARGET_LIB=AirLib\deps\rpclib\lib\x64
if NOT exist %RPCLIB_TARGET_LIB% mkdir %RPCLIB_TARGET_LIB%
set RPCLIB_TARGET_INCLUDE=AirLib\deps\rpclib\include
if NOT exist %RPCLIB_TARGET_INCLUDE% mkdir %RPCLIB_TARGET_INCLUDE%
robocopy /MIR external\rpclib\include %RPCLIB_TARGET_INCLUDE%
robocopy /MIR external\rpclib\build\output\lib %RPCLIB_TARGET_LIB%

REM //---------- now we have all dependencies to compile AirSim.sln which will also compile MavLinkCom ----------
msbuild /p:Platform=x64 /p:Configuration=Debug AirSim.sln
if ERRORLEVEL 1 goto :buildfailed
msbuild /p:Platform=x64 /p:Configuration=Release AirSim.sln 
if ERRORLEVEL 1 goto :buildfailed

REM //---------- copy binaries and include for MavLinkCom in deps ----------
set MAVLINK_TARGET_LIB=AirLib\deps\MavLinkCom\lib
if NOT exist %MAVLINK_TARGET_LIB% mkdir %MAVLINK_TARGET_LIB%
set MAVLINK_TARGET_INCLUDE=AirLib\deps\MavLinkCom\include
if NOT exist %MAVLINK_TARGET_INCLUDE% mkdir %MAVLINK_TARGET_INCLUDE%
robocopy /MIR MavLinkCom\include %MAVLINK_TARGET_INCLUDE%
robocopy /MIR MavLinkCom\lib %MAVLINK_TARGET_LIB%

REM //---------- all our output goes to Unreal/Plugin folder ----------
if NOT exist Unreal\Plugins\AirSim\Source\AirLib mkdir Unreal\Plugins\AirSim\Source\AirLib
robocopy /MIR AirLib Unreal\Plugins\AirSim\Source\AirLib  /XD temp *. /njh /njs /ndl /np

REM //---------- done building ----------
goto :eof

:buildfailed
chdir /d %ROOT_DIR% 
echo #### Build failed 1>&2
goto :eof

:installcmake
if NOT EXIST cmake-3.7.2-win64-x64 call :downloadcmake
set PATH=%PATH%;%ROOT_DIR%\cmake-3.7.2-win64-x64\bin;
goto :eof

:downloadcmake
echo CMake was not found, so we are installing it for you... 
%ROOT_DIR%\tools\httpget "https://cmake.org/files/v3.7/cmake-3.7.2-win64-x64.zip"
if ERRORLEVEL 1 goto :cmakefailed
echo Decompressing cmake-3.7.2-win64-x64.zip...
%ROOT_DIR%\tools\unzip "cmake-3.7.2-win64-x64.zip"
if ERRORLEVEL 1 goto :cmakefailed
del cmake-3.7.2-win64-x64.zip
goto :eof

:cmakefailed
echo CMake install failed, please install cmake manually from https://cmake.org/ 1>&2
exit /b 1
