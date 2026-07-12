param(
  [string]$Version = 'v1.2.0',
  [string]$DestRel = 'Unreal/Plugins/AirSim/Content/VehicleAdv',
  [switch]$Force
)

$ErrorActionPreference = 'Stop'

function Ensure-Tls12 {
  try {
    [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
  } catch {}
}

function New-TempDir([string]$Prefix='suv_download_tmp'){
  $base = Join-Path -Path (Get-Location) -ChildPath $Prefix
  if (Test-Path $base) { Remove-Item -Recurse -Force $base }
  New-Item -ItemType Directory -Force -Path $base | Out-Null
  return (Resolve-Path $base).Path
}

Write-Host '--- Downloading AirSim SUV assets ---' -ForegroundColor Cyan
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$dest = Join-Path $repoRoot $DestRel
$suvDir = Join-Path $dest 'SUV'
$url = "https://github.com/Microsoft/AirSim/releases/download/$Version/car_assets.zip"

Write-Host "Repo root: $repoRoot"
Write-Host "Destination: $dest"
Write-Host "URL: $url"

if ((Test-Path $suvDir) -and -not $Force){
  Write-Host "SUV folder already exists: $suvDir (use -Force to overwrite)" -ForegroundColor Yellow
  Write-Host 'Skipping download.'
  exit 0
}

Ensure-Tls12
$tmp = New-TempDir
$zip = Join-Path $tmp 'car_assets.zip'

Write-Host 'Downloading...' -ForegroundColor Cyan
Invoke-WebRequest -Uri $url -OutFile $zip -UseBasicParsing

Write-Host 'Extracting...' -ForegroundColor Cyan
if (-not (Test-Path $dest)) { New-Item -ItemType Directory -Force -Path $dest | Out-Null }
if (Test-Path $suvDir) { Remove-Item -Recurse -Force $suvDir }
Expand-Archive -Path $zip -DestinationPath $dest -Force

Remove-Item -Recurse -Force $tmp

if (Test-Path $suvDir){
  Write-Host "Done. Extracted to: $suvDir" -ForegroundColor Green
  exit 0
} else {
  Write-Host 'Extraction finished but SUV folder not found. Please verify manually.' -ForegroundColor Yellow
  exit 1
}

