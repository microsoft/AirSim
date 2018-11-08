@echo off
REM //---------- set up variable ----------
setlocal
set ROOT_DIR=%~dp0

REM root folder for repos
set RepoRoot=%1
set CommitMessage=%2

IF "%CommitMessage%"=="" (
	echo "CommitMessage needs to be provided"
	goto :failed
)

IF NOT EXIST %RepoRoot% (
	echo "RepoRoot needs to be provided"
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
@echo Now doing "%%x"
cd /d %RepoRoot%
cd "%%x"
git add -A
git commit -m %CommitMessage%
git push
cd ..
       )


goto :done

:failed
@echo "Error occured"
@echo Usage: gitcommitall <repo root> <commit message>
cd "%ROOT_DIR%"
exit /b 1

:done
cd "%ROOT_DIR%"
if "%1"=="" pause