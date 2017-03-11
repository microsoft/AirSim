
pushd %~d0
cd ..\..

rem update rpclib submodule
git submodule update --init --recursive

WHERE cmake >nul 2>nul
IF %ERRORLEVEL% NEQ 0 (
	echo cmake was not found! First install the latest version from https://cmake.org/.
	goto :buildfailed
)

IF NOT EXIST external\rpclib\build mkdir external\rpclib\build
pushd external\rpclib\build
cmake -G"Visual Studio 14 2015 Win64" ..
cmake --build .
cmake --build . --config Release
if ERRORLEVEL 1 goto :buildfailed
popd

set RPCLIB_TARGET_LIB=AirLib\deps\rpclib\lib\x64
if NOT exist %RPCLIB_TARGET_LIB% mkdir %RPCLIB_TARGET_LIB%

set RPCLIB_TARGET_INCLUDE=AirLib\deps\rpclib\include
if NOT exist %RPCLIB_TARGET_INCLUDE% mkdir %RPCLIB_TARGET_INCLUDE%

robocopy /MIR external\rpclib\include %RPCLIB_TARGET_INCLUDE%
robocopy /MIR external\rpclib\build\output\lib %RPCLIB_TARGET_LIB%

echo ### rpclib build complete
goto :eof

:buildfailed
chdir /d %ROOT_DIR% 
echo #### Build failed
goto :eof