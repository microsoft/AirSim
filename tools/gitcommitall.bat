REM @echo off
REM //---------- set up variable ----------
setlocal
set ROOT_DIR=%~dp0

REM root folder for repos
set RepoRoot=%1

IF NOT EXIST %RepoRoot% (
	echo "Specify root directory for repo as argument!"
	goto :failed
)

for %%x in (
1919Presentation
Africa
AirSimEnvNH
AncientRome
ApartmentInterior
B99
B99_b
CityEnviron
Coastline
DowntownCar
Forest
LandscapeMountains
Manhatten_Test
ModularCity
Plains
SimpleMaze
TalkingHeads
TrapCamera
Warehouse
ZhangJiaJie
       ) do (
echo Now doing "%%x"
cd %RepoRoot%
pwd
cd "%%x"
git add -A
git commit -m "By gitcomitall.bat"
git push
cd ..
       )


goto :done

:failed
echo "Error occured while packaging"
echo "Usage: package.bat <path\to\output> <path to Engine\Build\BatchFiles> <UE Version>"
cd "%ROOT_DIR%"
exit /b 1

:done
cd "%ROOT_DIR%"
if "%1"=="" pause