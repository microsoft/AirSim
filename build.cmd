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
    echo oh oh... You need to run this command from x64 Native Tools Command Prompt for VS 2019.
    goto :buildfailed_nomsg
)
if "%VisualStudioVersion%" lss "16.0" (
    echo(
    echo Hello there! We just upgraded AirSim to Unreal Engine 4.24 and Visual Studio 2019.
    echo Here are few easy steps for upgrade so everything is new and shiny:
    echo https://github.com/Microsoft/AirSim/blob/master/docs/unreal_upgrade.md
    goto :buildfailed_nomsg
)

if "%1"=="" goto noargs
if "%1"=="--no-full-poly-car" set "noFullPolyCar=y"
if "%1"=="--Debug" set "buildMode=debug"
if "%1"=="--Release" set "buildMode=release"
if "%1"=="--RelWithDebInfo" set "buildMode=RelWithDebInfo"

if "%2"=="" goto noargs
if "%2"=="--Debug" set "buildMode=debug"
if "%2"=="--Release" set "buildMode=release"
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
%powershell% ./install.ps1 -NoFullPolyCar %noFullPolyCar% -BuildMode %buildMode%
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

REM //---------- now we have all dependencies to compile AirSim.sln which will also compile MavLinkCom ----------
if "%buildMode%" == "" set "buildMode=debug"

set "buildDir=./build/build/%BuildMode%"
cmake -S./cmake -B"%buildDir%" -DCMAKE_INSTALL_PREFIX="./install_%BuildMode%" ^
  -GNinja ^
  -DFORCE_INSTALL_3RDPARTY=ON ^
  -DCMAKE_C_COMPILER=clang-cl ^
  -DCMAKE_CXX_COMPILER=clang-cl ^
  -DBUILD_TESTS=ON ^
  -DBUILD_EXAMPLES=ON ^
  -DCMAKE_BUILD_TYPE="%BuildMode%"

cmake --build %buildDir% --config %BuildMode%
if ERRORLEVEL 1 goto :buildfailed

cmake --install %buildDir% --config %BuildMode%

robocopy ".\install_%BuildMode%" ".\Unreal\Plugins\AirSim\Source\AirLib\%BuildMode%" /MIR /xd bin share cmake

REM //---------- done building ----------
exit /b 0

:buildfailed
echo(
echo #### Build failed - see messages above. 1>&2

:buildfailed_nomsg
chdir /d %ROOT_DIR% 
exit /b 1