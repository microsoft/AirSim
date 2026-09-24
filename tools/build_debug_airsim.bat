@echo off
setlocal
REM Initialize VS 2022 developer environment (x64)
call "D:\VS2022\Community\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64
if ERRORLEVEL 1 (
  echo Failed to initialize VS DevCmd. >&2
  exit /b 1
)

cd /d E:\Project\AI\python\project\AirSimMy
call build.cmd --Debug --no-full-poly-car
exit /b %ERRORLEVEL%

