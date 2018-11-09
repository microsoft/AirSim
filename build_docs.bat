@echo off
REM //---------- set up variable ----------
setlocal
set ROOT_DIR=%~dp0
pushd %~dp0

set BUILD_DIR=%ROOT_DIR%temp\docs_build
rem mdir /s /q %BUILD_DIR%
robocopy "docs" "%BUILD_DIR%\doc_root\docs" /MIR /njh /njs /ndl /np /nfl /r:0
robocopy "." "%BUILD_DIR%\doc_root" *.md /njh /njs /ndl /np /nfl /r:0
robocopy "%BUILD_DIR%\doc_root\docs" "%BUILD_DIR%" mkdocs.yml /MOV /njh /njs /ndl /np /nfl /r:0

REM Copy of all folders is no needed as we use absolute URLs now
REM for /d %%x in (
REM     "%ROOT_DIR%*"
REM        ) do (
REM     cd /d "%BUILD_DIR%\doc_root"
REM     IF "%%~nx"=="" (
REM         REM Do nothing
REM     ) ELSE (
REM         IF NOT EXIST "%%~nx" mklink /D "%%~nx" "%ROOT_DIR%%%~nx"
REM     )
REM )

cd /d %BUILD_DIR%

if "%1"=="no_serve" goto PromptYN
mkdocs serve

:PromptYN
REM if exist "%SystemRoot%\System32\choice.exe" goto UseChoice
setlocal EnableExtensions EnableDelayedExpansion
:UseSetPrompt
set "PromptMessage=Do you want to commit to GitHub Pages [Y/N]? "
set "UserChoice=N"
set /P "UserChoice=%PromptMessage%"
set "UserChoice=!UserChoice: =!"
if /I "!UserChoice!" == "N" endlocal & goto :AnswerNo
if /I not "!UserChoice!" == "Y" goto UseSetPrompt
endlocal
goto AnswerYes
REM :UseChoice
REM %SystemRoot%\System32\choice.exe /C YN /N /M "%PromptMessage%"
REM if errorlevel 2 goto :AnswerNo
REM goto AnswerYes

:AnswerYes
@echo Building and commiting to gh-pages branch...
mkdocs build
git checkout gh-pages
cd /d %BUILD_DIR%
robocopy "%BUILD_DIR%\build" "%ROOT_DIR%docs" /MIR /njh /njs /ndl /np /nfl /r:0

goto success

:AnswerNo
@echo No commits were done.
goto success

:success
@echo "Task completed."
goto end

:failed
@echo "Task has failed."
goto end

:end
popd
