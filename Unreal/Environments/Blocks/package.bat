@echo off
REM //---------- set up variable ----------
setlocal
set ROOT_DIR=%~dp0

set OutPath=%1
set ToolPath=%2
set UEVer=%3

if "%UEVer%"=="" set "UEVer=4.24"

set "_ToolPath=%PROGRAMFILES%\Epic Games\UE_%UEVer%\Engine\Build\BatchFiles"
if "%ToolPath%"=="" set ToolPath=%_ToolPath%
if "%ToolPath%"=="""" set ToolPath=%_ToolPath%

IF NOT EXIST "%ToolPath%" (
	echo "Unreal Engine build path %ToolPath% was not found"
	goto :failed
)

if "%OutPath%"=="" set "OutPath=D:\AirSimBuilds"
IF NOT EXIST "%ToolPath%" (
	echo "Package output path %OutPath% was not found"
	goto :failed
)

echo Using ToolPath = %ToolPath%
echo Using OutPath = %OutPath%

for %%f in (*.uproject) do (
		echo Packaging: %%f
		
		"%ToolPath%\Build" "%%~nfEditor" Win64 Development -WarningsAsErrors "%cd%\%%f"
		if ERRORLEVEL 1 goto :failed
		
		REM "%ToolPath%\RunUAT" -ScriptsForProject="%cd%\%%f" BuildCookRun -installed -nop4 -project="%cd%\%%f" -cook -stage -archive -archivedirectory="%OutPath%" -package -clientconfig=Development -ue4exe=UE4Editor-Cmd.exe -compressed -pak -prereqs -nodebuginfo -targetplatform=Win64 -build -utf8output -nocompile -nocompileeditor 
		
		REM "%ToolPath%\RunUAT" BuildCookRun -project="%cd%\%%f" -noP4 -platform=Win64 -clientconfig=Development -serverconfig=Development -cook -rocket -allmaps -build -stage -NoCompile -nocompileeditor -pak -archive -archivedirectory="%OutPath%"
		
		rmdir /s /q "%OutPath%\%%~nf"
		
		"%ToolPath%\RunUAT" BuildCookRun -project="%cd%\%%f" -noP4 -platform=Win64 -clientconfig=Development -cook -build -stage -pak -archive -archivedirectory="%OutPath%"  -utf8output -compressed -prereqs
		if ERRORLEVEL 1 goto :failed
		
		move "%OutPath%\WindowsNoEditor" "%OutPath%\%%~nf"
		if ERRORLEVEL 1 goto :failed
		
		@echo off
		echo start %%~nf -windowed> "%OutPath%\%%~nf\run.bat"
		if ERRORLEVEL 1 goto :failed
)

goto :done

:failed
echo "Error occured while packaging"
echo "Usage: package.bat <path\to\output> <path to Engine\Build\BatchFiles> <UE Version>"
exit /b 1

:done
if "%1"=="" pause

	

