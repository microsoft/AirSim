@echo off
REM //---------- set up variable ----------
setlocal
set ROOT_DIR=%~dp0

REM // Check command line arguments
set "noFullPolyCar="
set "buildMode="

REM //check VS version
if "%VisualStudioVersion%" == "" (
    echo(
    echo oh oh... You need to run this command from x64 Native Tools Command Prompt for VS 2022.
    goto :buildfailed_nomsg
)
if "%VisualStudioVersion%" lss "17.0" (
    echo(
    echo Hello there! We just upgraded AirSim to Unreal Engine 4.27 and Visual Studio 2022.
    echo Here are few easy steps for upgrade so everything is new and shiny:
    echo https://github.com/Microsoft/AirSim/blob/main/docs/unreal_upgrade.md
    goto :buildfailed_nomsg
)

if "%1"=="" goto noargs
if "%1"=="--no-full-poly-car" set "noFullPolyCar=y"
if "%1"=="--Debug" set "buildMode=Debug"
if "%1"=="--Release" set "buildMode=Release"
if "%1"=="--RelWithDebInfo" set "buildMode=RelWithDebInfo"

if "%2"=="" goto noargs
if "%2"=="--Debug" set "buildMode=Debug"
if "%2"=="--Release" set "buildMode=Release"
if "%2"=="--RelWithDebInfo" set "buildMode=RelWithDebInfo"

:noargs

set powershell=powershell
where powershell > nul 2>&1
if ERRORLEVEL 1 goto :pwsh
echo found Powershell && goto start
:pwsh
set powershell=pwsh
where pwsh > nul 2>&1
if ERRORLEVEL 1 goto :nopwsh
set PWSHV7=1
echo found pwsh && goto start
:nopwsh
echo Powershell or pwsh not found, please install it.
goto :eof

:start
chdir /d %ROOT_DIR% 

REM //---------- Check cmake version ----------
CALL check_cmake.bat
if ERRORLEVEL 1 (
  CALL check_cmake.bat
  if ERRORLEVEL 1 (
    echo(
    echo ERROR: cmake was not installed correctly, we tried.
    goto :buildfailed
  )
)

REM //---------- get rpclib ----------
IF NOT EXIST external\rpclib mkdir external\rpclib

set RPC_VERSION_FOLDER=rpclib-2.3.0
IF NOT EXIST external\rpclib\%RPC_VERSION_FOLDER% (
    REM //leave some blank lines because %powershell% shows download banner at top of console
    ECHO(
    ECHO(   
    ECHO(   
    ECHO *****************************************************************************************
    ECHO Downloading rpclib
    ECHO *****************************************************************************************
    @echo on
    if "%PWSHV7%" == "" (
        %powershell% -command "& { [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12; iwr https://github.com/rpclib/rpclib/archive/v2.3.0.zip -OutFile external\rpclib.zip }"
    ) else (
        %powershell% -command "iwr https://github.com/rpclib/rpclib/archive/v2.3.0.zip -OutFile external\rpclib.zip"
    )
    @echo off
    
    REM //remove any previous versions
    rmdir external\rpclib /q /s

    %powershell% -command "Expand-Archive -Path external\rpclib.zip -DestinationPath external\rpclib"
    del external\rpclib.zip /q
    
    REM //Fail the build if unable to download rpclib
    IF NOT EXIST external\rpclib\%RPC_VERSION_FOLDER% (
        ECHO Unable to download rpclib, stopping build
        goto :buildfailed
    )
)

REM //---------- Build rpclib ------------
ECHO Starting cmake to build rpclib...
IF NOT EXIST external\rpclib\%RPC_VERSION_FOLDER%\build mkdir external\rpclib\%RPC_VERSION_FOLDER%\build
cd external\rpclib\%RPC_VERSION_FOLDER%\build
cmake -G"Visual Studio 17 2022" ..

if "%buildMode%" == "" (
cmake --build . 
cmake --build . --config Release
) else (
cmake --build . --config %buildMode%
)

if ERRORLEVEL 1 goto :buildfailed
chdir /d %ROOT_DIR% 

REM //---------- copy rpclib binaries and include folder inside AirLib folder ----------
set RPCLIB_TARGET_LIB=AirLib\deps\rpclib\lib\x64
if NOT exist %RPCLIB_TARGET_LIB% mkdir %RPCLIB_TARGET_LIB%
set RPCLIB_TARGET_INCLUDE=AirLib\deps\rpclib\include
if NOT exist %RPCLIB_TARGET_INCLUDE% mkdir %RPCLIB_TARGET_INCLUDE%
robocopy /MIR external\rpclib\%RPC_VERSION_FOLDER%\include %RPCLIB_TARGET_INCLUDE%

if "%buildMode%" == "" (
robocopy /MIR external\rpclib\%RPC_VERSION_FOLDER%\build\Debug %RPCLIB_TARGET_LIB%\Debug
robocopy /MIR external\rpclib\%RPC_VERSION_FOLDER%\build\Release %RPCLIB_TARGET_LIB%\Release
) else (
robocopy /MIR external\rpclib\%RPC_VERSION_FOLDER%\build\%buildMode% %RPCLIB_TARGET_LIB%\%buildMode%
)

REM //---------- get High PolyCount SUV Car Model ------------
IF NOT EXIST Unreal\Plugins\AirSim\Content\VehicleAdv mkdir Unreal\Plugins\AirSim\Content\VehicleAdv
IF NOT EXIST Unreal\Plugins\AirSim\Content\VehicleAdv\SUV\v1.2.0 (
    IF NOT DEFINED noFullPolyCar (
        REM //leave some blank lines because %powershell% shows download banner at top of console
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
        REM %powershell% -command "& { Start-BitsTransfer -Source https://github.com/Microsoft/AirSim/releases/download/v1.2.0/car_assets.zip -Destination suv_download_tmp\car_assets.zip }"
        REM %powershell% -command "& { (New-Object System.Net.WebClient).DownloadFile('https://github.com/Microsoft/AirSim/releases/download/v1.2.0/car_assets.zip', 'suv_download_tmp\car_assets.zip') }"
        if "%PWSHV7%" == "" (
            %powershell% -command "& { [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12; iwr https://github.com/Microsoft/AirSim/releases/download/v1.2.0/car_assets.zip -OutFile suv_download_tmp\car_assets.zip }"
        ) else (
            %powershell% -command "iwr https://github.com/Microsoft/AirSim/releases/download/v1.2.0/car_assets.zip -OutFile suv_download_tmp\car_assets.zip"
        )
        @echo off
        rmdir /S /Q Unreal\Plugins\AirSim\Content\VehicleAdv\SUV
        %powershell% -command "Expand-Archive -Path suv_download_tmp\car_assets.zip -DestinationPath Unreal\Plugins\AirSim\Content\VehicleAdv"
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
    if "%PWSHV7%" == "" (
        %powershell% -command "& { [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12; iwr https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip -OutFile eigen3.zip }"
    ) else (
        %powershell% -command "iwr https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip -OutFile eigen3.zip"
    )
    %powershell% -command "Expand-Archive -Path eigen3.zip -DestinationPath AirLib\deps"
    %powershell% -command "Move-Item -Path AirLib\deps\eigen* -Destination AirLib\deps\del_eigen"
    REM move AirLib\deps\eigen* AirLib\deps\del_eigen
    mkdir AirLib\deps\eigen3
    move AirLib\deps\del_eigen\Eigen AirLib\deps\eigen3\Eigen
    rmdir /S /Q AirLib\deps\del_eigen
    del eigen3.zip
)
IF NOT EXIST AirLib\deps\eigen3 goto :buildfailed


REM //---------- now we have all dependencies to compile AirSim.sln which will also compile MavLinkCom ----------
if "%buildMode%" == "" (
msbuild -maxcpucount:12 /p:Platform=x64 /p:Configuration=Debug AirSim.sln
if ERRORLEVEL 1 goto :buildfailed
msbuild -maxcpucount:12 /p:Platform=x64 /p:Configuration=Release AirSim.sln 
if ERRORLEVEL 1 goto :buildfailed
) else (
msbuild -maxcpucount:12 /p:Platform=x64 /p:Configuration=%buildMode% AirSim.sln
if ERRORLEVEL 1 goto :buildfailed
)

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
copy /y AirSim.props Unreal\Plugins\AirSim\Source\AirLib

REM //---------- update all environments ----------
FOR /D %%E IN (Unreal\Environments\*) DO (
    cd %%E
    call .\update_from_git.bat ..\..\..
    cd ..\..\..
)

REM //---------- done building ----------
exit /b 0

:buildfailed
echo(
echo #### Build failed - see messages above. 1>&2

:buildfailed_nomsg
chdir /d %ROOT_DIR% 
exit /b 1



