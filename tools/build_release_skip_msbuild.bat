@echo off
setlocal
call "D:\VS2022\Community\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64
if ERRORLEVEL 1 (
  echo Failed to initialize VS DevCmd. >&2
  exit /b 1
)
cd /d E:\Project\AI\python\project\AirSimMy
call build.cmd --Release --no-full-poly-car --skip-msbuild
exit /b %ERRORLEVEL%

