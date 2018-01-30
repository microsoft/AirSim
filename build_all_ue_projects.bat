rem @echo off
REM //---------- set up variable ----------
setlocal
set ROOT_DIR=%~dp0

REM root folder containing all Unreal projects you want to build
set UnrealProjsPath=%1
REM Output folder where the builds would be stored
set OutputPath=%2
REM folder containing AirSim repo
set AirSimPath=%3

REM set defaults if ars are not supplied
if "%UnrealProjsPath%"=="" set "UnrealProjsPath=D:\vso\msresearch\Theseus"
if "%AirSimPath%"=="" set "AirSimPath=C:\GitHubSrc\AirSim"
if "%OutputPath%"=="" set "OutputPath=%ROOT_DIR%build"

REM re-build airsim
cd /D "%AirSimPath%"
CALL clean
CALL build
if ERRORLEVEL 1 goto :failed

cd /D "%ROOT_DIR%"

IF NOT EXIST "%OutputPath%" mkdir "%OutputPath%"

call:doOneProject "ZhangJiaJie"
call:doOneProject "AirSimEnvNH"
call:doOneProject "AncientRome"
call:doOneProject "CityEnviron"
call:doOneProject "DowntownCar"
call:doOneProject "LandscapeMountains"
call:doOneProject "ModulerCity"
call:doOneProject "Africa_001" "Africa"

goto :done

:doOneProject
if "%~2"=="" (
 cd /D "%UnrealProjsPath%\%~1"
) else (
 cd "%UnrealProjsPath%\%~2"
)
robocopy "%AirSimPath%\Unreal\Environments\Blocks" . update_from_git.bat  /njh /njs /ndl /np

CALL update_from_git.bat "%AirSimPath%"
if ERRORLEVEL 1 goto :failed

CALL package.bat "%OutputPath%"
if ERRORLEVEL 1 goto :failed

cd "%ROOT_DIR%"
GOTO:EOF

goto :done

:failed
echo "Error occured while packaging"
echo "Usage: package.bat <path\to\output> <path to Engine\Build\BatchFiles>"
exit /b 1

:done
if "%1"=="" pause