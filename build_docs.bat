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
REM REM if exist "%SystemRoot%\System32\choice.exe" goto UseChoice
REM setlocal EnableExtensions EnableDelayedExpansion
REM :UseSetPrompt
REM set "PromptMessage=Do you want to commit to GitHub Pages [Y/N]? "
REM set "UserChoice=N"
REM set /P "UserChoice=%PromptMessage%"
REM set "UserChoice=!UserChoice: =!"
REM if /I "!UserChoice!" == "N" endlocal & goto :AnswerNo
REM if /I not "!UserChoice!" == "Y" goto UseSetPrompt
REM endlocal
REM goto AnswerYes
REM REM :UseChoice
REM REM %SystemRoot%\System32\choice.exe /C YN /N /M "%PromptMessage%"
REM REM if errorlevel 2 goto :AnswerNo
REM REM goto AnswerYes

REM :AnswerYes
REM @echo Building and commiting to gh-pages branch...
mkdocs build
robocopy "%BUILD_DIR%\doc_root\docs\images" "%BUILD_DIR%\build\images" /MIR /njh /njs /ndl /np /nfl /r:0
robocopy "%BUILD_DIR%\doc_root\docs\misc" "%BUILD_DIR%\build\misc" /MIR /njh /njs /ndl /np /nfl /r:0
robocopy "%BUILD_DIR%\doc_root\docs\paper" "%BUILD_DIR%\build\paper" /MIR /njh /njs /ndl /np /nfl /r:0

@echo Next Steps:
@echo git checkout gh-pages
@echo Copy "%BUILD_DIR%\build" to root
@echo Push gh-pages

REM batch file will get deleted so must execute commands in one line
REM git checkout gh-pages && cd /d %BUILD_DIR% && robocopy "%BUILD_DIR%\build" "%ROOT_DIR%docs" /MIR /njh /njs /ndl /np /nfl /r:0


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
