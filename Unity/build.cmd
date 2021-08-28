echo off

if "%VisualStudioVersion%" == "" (
    echo(
    echo oh oh... You need to run this command from x64 Native Tools Command Prompt for VS 2019.
    goto :buildfailed
)
if "%VisualStudioVersion%" lss "16.0" (
    echo(
    echo Visual Studio 2019 build tools are required.
    goto :buildfailed
)

REM Visual Studio Generator doesn't work for some unknown reason, Ninja and clang are preferred
cmake ^
 -G "Ninja"^
 -DCMAKE_C_COMPILER=clang-cl^
 -DCMAKE_CXX_COMPILER=clang-cl^
 -S./AirLibWrapper/AirsimWrapper^
 -B./build/build^
 -DCMAKE_INSTALL_BINDIR=.^
 -DCMAKE_INSTALL_PREFIX=./UnityDemo/Assets/Plugins^
 -DCMAKE_BUILD_TYPE=Release
cmake --build  ./build/build --config Release --target AirsimWrapper
cmake --install ./build/build --config Release

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