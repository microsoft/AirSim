echo off

if "%VisualStudioVersion%" == "" (
    echo(
    echo oh oh... You need to run this command from x64 Native Tools Command Prompt for VS 2019.
    goto :buildfailed
)
if "%VisualStudioVersion%" lss "16.0" (
    echo(
    echo Hello there! We just upgraded AirSim to Unreal Engine 4.24 and Visual Studio 2019.
    echo Here are few easy steps for upgrade so everything is new and shiny:
    echo https://github.com/Microsoft/AirSim/blob/master/docs/unreal_upgrade.md
    goto :buildfailed
)

cmake -S./AirLibWrapper/AirsimWrapper^
 -B./build/build^
 -GNinja^
 -DCMAKE_C_COMPILER=clang-cl^
 -DCMAKE_CXX_COMPILER=clang-cl^
 -DCMAKE_INSTALL_BINDIR=.^
 -DCMAKE_INSTALL_PREFIX=./build/install
cmake --build  ./build/build --config Release --target AirsimWrapper
cmake --install ./build/build --config Release --prefix ./UnityDemo/Assets/Plugins

if ERRORLEVEL 1 goto :buildfailed

set ROOT=%CD%
chdir UnityDemo/Assets/Plugins
rmdir /Q /S include
rmdir /Q /S lib
rmdir /Q /S share
chdir %ROOT%

REM //---------- done building ----------
exit /b 0

:buildfailed
echo(
echo #### Build failed - see messages above. 1>&2
exit /b 1

echo on