

function Get-ScriptDirectory
{
    $Invocation = (Get-Variable MyInvocation -Scope 1).Value
    Split-Path $Invocation.MyCommand.Path
}

$scriptDir = Get-ScriptDirectory
Set-Location -Path $scriptDir

function HttpGet($uri)
{
    $rc = & $scriptDir\tools\httpget $uri;
    echo "httpget returned $rc"
}
function Unzip($zipFile)
{
    $rc = & $scriptDir\tools\unzip $zipFile;
    echo "unzip returned $rc";
}
function UnzipToDir($zipFile, $dir)
{
    echo "$scriptDir\tools\unzip $zipFile -d $dir"
    $rc = & $scriptDir\tools\unzip $zipFile "-d" $dir
    echo "unzip returned $rc";
}

function InstallCMake()
{
    echo "Checking cmake install..."
    if (-Not (Get-Command "cmake.exe" -ErrorAction SilentlyContinue))
    { 
        if (-Not (Test-Path "$scriptDir\cmake-3.7.2-win64-x64"))  {
            HttpGet "https://cmake.org/files/v3.7/cmake-3.7.2-win64-x64.zip"; 
            echo "Decompressing cmake...";
            Unzip "cmake-3.7.2-win64-x64.zip";
            Remove-Item cmake-3.7.2-win64-x64.zip;
        }
        $env:Path = "$env:Path;$scriptDir\cmake-3.7.2-win64-x64\bin\";
    }
    if (-Not (Get-Command "cmake.exe" -ErrorAction SilentlyContinue))
    {
        throw "Install cmake failed!!!"
    }
}

$start = Get-Date
echo "======================= Build Started $start ====================";

git submodule update --init --recursive;
InstallCMake;

if (-Not (Test-Path "external\rpclib\build")) {
    mkdir "external\rpclib\build"
}
Set-Location -Path "external\rpclib\build"

cmake -G"Visual Studio 14 2015 Win64" ..
cmake --build .
cmake --build . --config Release

Set-Location -Path $scriptDir 

$env:RPCLIB_TARGET_LIB="AirLib\deps\rpclib\lib\x64";
if (-Not (Test-Path "$env:RPCLIB_TARGET_LIB")) {
    mkdir $env:RPCLIB_TARGET_LIB
}
$env:RPCLIB_TARGET_INCLUDE="AirLib\deps\rpclib\include"
if (-Not (Test-Path "$env:RPCLIB_TARGET_INCLUDE")) {
    mkdir $env:RPCLIB_TARGET_INCLUDE
}

robocopy /MIR external\rpclib\include  $env:RPCLIB_TARGET_INCLUDE
robocopy /MIR external\rpclib\build\output\lib $env:RPCLIB_TARGET_LIB

msbuild /p:Platform=x64 /p:Configuration=Debug AirSim.sln
msbuild /p:Platform=x64 /p:Configuration=Release AirSim.sln 

$env:MAVLINK_TARGET_LIB="AirLib\deps\MavLinkCom\lib"
if (-Not (Test-Path "$env:MAVLINK_TARGET_LIB")) {
    mkdir $env:MAVLINK_TARGET_LIB
}
$env:MAVLINK_TARGET_INCLUDE="AirLib\deps\MavLinkCom\include"
if (-Not (Test-Path "$env:MAVLINK_TARGET_INCLUDE")) {
    mkdir $env:MAVLINK_TARGET_INCLUDE
}
$env:UNREAL_AIRLIB_TARGET_DIR="Unreal\Plugins\AirSim\Source\AirLib"
if (-Not (Test-Path "$env:UNREAL_AIRLIB_TARGET_DIR")) {
    mkdir "Unreal\Plugins\AirSim\Source\AirLib"
}
robocopy /MIR MavLinkCom\include $env:MAVLINK_TARGET_INCLUDE
robocopy /MIR MavLinkCom\lib $env:MAVLINK_TARGET_LIB
robocopy /MIR AirLib $env:UNREAL_AIRLIB_TARGET_DIR  /XD temp

$end = Get-Date
$diff = $end - $start
echo "======================= Build Completed in $diff ====================";
