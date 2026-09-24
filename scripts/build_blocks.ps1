param(
    [string]$EngineRoot = 'D:\UE_5.3',
    [string]$Config = 'Development',
    [string]$Platform = 'Win64'
)

$ErrorActionPreference = 'Stop'

# Resolve project path relative to repo root
$repoRoot = Split-Path -Parent $PSScriptRoot
$uproject = Join-Path $repoRoot 'Unreal/Environments/Blocks/Blocks.uproject'
$uprojectPath = Resolve-Path $uproject

Write-Host "Using EngineRoot: $EngineRoot"
Write-Host "Project: $uprojectPath"
Write-Host "Building Target: BlocksEditor $Platform $Config"

$buildBat = Join-Path $EngineRoot 'Engine/Build/BatchFiles/Build.bat'
if (!(Test-Path $buildBat)) {
    throw "Build.bat not found at $buildBat"
}

& $buildBat 'BlocksEditor' $Platform $Config $uprojectPath '-WaitMutex' '-FromMsBuild'
if ($LASTEXITCODE -ne 0) {
    throw "Build failed with exit code $LASTEXITCODE"
}

Write-Host 'Build succeeded.'
