@echo off
cd %~dp0

echo Uploading ClickOnce installer for Px4LogViewer
AzurePublishClickOnce %~dp0publish downloads/Px4LogViewer "%LOVETTSOFTWARE_STORAGE_CONNECTION_STRING%"
if ERRORLEVEL 1 goto :eof
