@echo off
REM //---------- set up variable ----------
setlocal
set ROOT_DIR=%~dp0

rd /s/q external
git clean -ffdx
git pull
build.cmd
