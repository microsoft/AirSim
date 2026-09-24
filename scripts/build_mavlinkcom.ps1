param(
    [ValidateSet('Debug','Release')][string]$Configuration = 'Release',
    [ValidateSet('x64','Win32')][string]$Platform = 'x64',
    [string]$Toolset = 'v143',                    # PlatformToolset override (optional)
    [string]$WindowsSDK = '',                     # e.g. 10.0.22621.0 (optional)
    [string]$VSInstallPath = '',                  # VS root (optional)
    [switch]$Rebuild,
    [switch]$DisableTracker = $true,              # helps avoid TRK0002
    [switch]$CopyToAirSim = $true                 # copy lib into AirSim deps after build
)

$ErrorActionPreference = 'Stop'

function Find-VSInstallPath {
    param([string]$Preferred)

    if ($Preferred -and (Test-Path $Preferred)) { return (Resolve-Path $Preferred).Path }

    $vswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
    if (Test-Path $vswhere) {
        $path = & $vswhere -latest -requires Microsoft.Component.MSBuild -property installationPath 2>$null
        if ($LASTEXITCODE -eq 0 -and $path) { return $path.Trim() }
    }

    $candidates = @(
        'C:\Program Files\Microsoft Visual Studio\2022\Community',
        'C:\Program Files\Microsoft Visual Studio\2022\Professional',
        'C:\Program Files\Microsoft Visual Studio\2022\Enterprise',
        'D:\VS2022\Community'
    )
    foreach ($c in $candidates) { if (Test-Path $c) { return $c } }
    throw 'Could not locate a Visual Studio 2022 installation. Set -VSInstallPath to override.'
}

function Get-MSBuildPath {
    param([string]$VSRoot)
    $paths = @(
        (Join-Path $VSRoot 'MSBuild\Current\Bin\MSBuild.exe'),
        (Join-Path $VSRoot 'MSBuild\Current\Bin\amd64\MSBuild.exe')
    )
    foreach ($p in $paths) { if (Test-Path $p) { return $p } }
    throw "MSBuild.exe not found under '$VSRoot'"
}

Write-Host "=== MavLinkCom build ===" -ForegroundColor Cyan
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$sln = Resolve-Path (Join-Path $repoRoot 'MavLinkCom\MavLinkCom.sln')
Write-Host "Repo root: $repoRoot"
Write-Host "Solution:  $sln"

$vsRoot = Find-VSInstallPath -Preferred $VSInstallPath
$msbuild = Get-MSBuildPath -VSRoot $vsRoot
Write-Host "VS Root:   $vsRoot"
Write-Host "MSBuild:   $msbuild"

$props = @(
    "/p:Configuration=$Configuration",
    "/p:Platform=$Platform",
    "/p:PlatformToolset=$Toolset"
)
if ($WindowsSDK) { $props += "/p:WindowsTargetPlatformVersion=$WindowsSDK" }
if ($DisableTracker) { $props += "/p:TrackFileAccess=false"; $props += "/p:UseMultiToolTask=false" }

$targets = if ($Rebuild) { '/t:Rebuild' } else { '/t:Build' }
$log = Join-Path $repoRoot 'MavLinkCom\build_mavlinkcom.binlog'

Write-Host "Building MavLinkCom ($Configuration|$Platform) ..."
& $msbuild $sln $targets '/m' '/nologo' '/v:m' "/bl:$log" @props
if ($LASTEXITCODE -ne 0) { throw "MSBuild failed with exit code $LASTEXITCODE. See $log" }

# Verify output and optionally copy into AirSim plugin deps
$platDir = if ($Platform -eq 'x64') { 'x64' } else { 'x86' }
$outLib = Join-Path $repoRoot "MavLinkCom\lib\$platDir\$Configuration\MavLinkCom.lib"
if (!(Test-Path $outLib)) { throw "Expected output not found: $outLib" }
Write-Host "Built:     $outLib" -ForegroundColor Green

if ($CopyToAirSim) {
    $dest = Join-Path $repoRoot "Unreal\Environments\Blocks\Plugins\AirSim\Source\AirLib\deps\MavLinkCom\lib\$platDir\$Configuration"
    New-Item -Force -ItemType Directory -Path $dest | Out-Null
    Copy-Item -Force $outLib $dest
    Write-Host "Copied to: $dest" -ForegroundColor Green
}

Write-Host "Done." -ForegroundColor Cyan
