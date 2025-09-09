param(
  [string]$PluginContent = 'Unreal/Plugins/AirSim/Content/VehicleAdv',
  [switch]$WriteSettings,
  [string]$SettingsPath = "$env:USERPROFILE/Documents/AirSim/settings.json"
)

$ErrorActionPreference = 'Stop'

function Test-PathSafe([string]$p){ try { Test-Path $p } catch { $false } }

Write-Host '--- AirSim SUV assets check ---' -ForegroundColor Cyan
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$contentRoot = Join-Path $root $PluginContent
$suvDir = Join-Path $contentRoot 'SUV'

Write-Host "PluginContent: $contentRoot"

$ok = $true
if (!(Test-PathSafe $suvDir)){
  Write-Warning "SUV content directory missing: $suvDir"
  $ok = $false
} else {
  $uassets = Get-ChildItem -Recurse -Filter *.uasset -ErrorAction SilentlyContinue $suvDir | Select-Object -First 1
  if (-not $uassets){
    Write-Warning "SUV directory exists but contains no .uasset files: $suvDir"
    $ok = $false
  } else {
    Write-Host "SUV assets found under: $suvDir" -ForegroundColor Green
  }
}

# Heuristics for slippery/non-slippery materials
$slippery = Get-ChildItem -Recurse -Filter *Slippery*.uasset -ErrorAction SilentlyContinue $contentRoot | Select-Object -First 1
$nonslip  = Get-ChildItem -Recurse -Filter *NonSlippery*.uasset -ErrorAction SilentlyContinue $contentRoot | Select-Object -First 1
if (-not $slippery){ Write-Warning 'Slippery physics material not found (search *Slippery*.uasset)'; $ok=$false } else { Write-Host "Slippery material: $($slippery.FullName)" }
if (-not $nonslip){ Write-Warning 'NonSlippery physics material not found (search *NonSlippery*.uasset)'; $ok=$false } else { Write-Host "NonSlippery material: $($nonslip.FullName)" }

if (-not $ok){
  Write-Host ''
  Write-Host 'Hint: Download high-poly SUV assets (car_assets.zip) from AirSim releases and extract to:' -ForegroundColor Yellow
  Write-Host "  $suvDir" -ForegroundColor Yellow
  Write-Host 'Reference: setup.sh (downloadHighPolySuv) in repo.' -ForegroundColor Yellow
}

if ($WriteSettings){
  $settings = @{
    SettingsVersion = 1
    SimMode = 'Car'
    PawnPaths = @{
      DefaultCar = @{
        # Use Unreal Editor: right-click asset -> Copy Reference, then paste below
        slippery_mat    = '/AirSim/VehicleAdv/Vehicle/Materials/Slippery.Slippery'
        non_slippery_mat= '/AirSim/VehicleAdv/Vehicle/Materials/NonSlippery.NonSlippery'
      }
    }
    Vehicles = @{ Car1 = @{ VehicleType = 'PhysXCar'; AutoCreate = $true } }
  } | ConvertTo-Json -Depth 6

  $dst = (Resolve-Path (Split-Path -Parent $SettingsPath) -ErrorAction SilentlyContinue)
  if (-not $dst){ New-Item -ItemType Directory -Force (Split-Path -Parent $SettingsPath) | Out-Null }
  Set-Content -Path $SettingsPath -Value $settings -Encoding UTF8
  Write-Host "Wrote settings to: $SettingsPath" -ForegroundColor Green
  Write-Host 'Note: Update slippery_mat/non_slippery_mat to match your asset references.' -ForegroundColor Yellow
}

if ($ok){ Write-Host 'SUV assets check passed.' -ForegroundColor Green } else { Write-Host 'SUV assets check incomplete.' -ForegroundColor Yellow }
