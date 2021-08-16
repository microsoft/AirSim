Param(
    [Parameter(
      Mandatory=$false, 
      HelpMessage="get High PolyCount SUV Car Model (default: true)")]
    [bool]
    $NoFullPolyCar=$true,
    [Parameter(
      Mandatory=$false,
      HelpMessage="cmake build type Release, Debug, RelWithDebInfo or MinRelSize (default: Release)")]
    [string]
    $BuildMode="Release",
    # Install Prefix
    [Parameter(
      Mandatory=$false,
      HelpMessage="where to install libraries (default: ./build/install/<BuildMode>)")]
    [string]
    $InstallPrefix="./build/install/$BuildMode",
    [Parameter(
      Mandatory=$false,
      HelpMessage="path to vswhere.exe (default: <env:ProgramFiles(x86)>/Microsoft Visual Studio/Installer/vswhere.exe) (windows only)")]
    [string]
    $VswherePath="${env:ProgramFiles(x86)}/Microsoft Visual Studio/Installer/vswhere.exe"
)

$isWindowsOS = [System.Runtime.InteropServices.RuntimeInformation]::IsOSPlatform([System.Runtime.InteropServices.OSPlatform]::Windows)

if ($isWindowsOS) {
  if (-not (Test-Path $VswherePath)) {
    Write-Error "vswhere not found, please specify this by VswherePath parameter"
  }

  $installationPath = &"$VswherePath" -latest -property installationPath
  if ($installationPath -and (test-path "$installationPath/Common7/Tools/vsdevcmd.bat")) {
    & "${env:COMSPEC}" /s /c "`"$installationPath/Common7/Tools/vsdevcmd.bat`" -no_logo && set" | foreach-object {
      $name, $value = $_ -split '=', 2
      set-content env:\"$name" $value
    }
  }
}

#---------- get High PolyCount SUV Car Model ------------
if (-not (Test-Path "./Unreal/Plugins/AirSim/Content/VehicleAdv")) {
  New-Item -Path "./Unreal/Plugins/AirSim/Content/VehicleAdv" -ItemType Directory
}

if (-not (Test-Path "./Unreal/Plugins/AirSim/Content/VehicleAdv/SUV/v1.2.0")) {
  if (-not $NoFullPolyCar) {
    "*****************************************************************************************
Downloading high-poly car assets.... The download is ~37MB and can take some time.
To install without this assets, re-run install.ps1 with the argument -NoFullPolyCar
*****************************************************************************************
    " | Out-Host
    if (Test-Path ./suv_download_tmp) {
      Remove-Item ./suv_download_tmp -Recurse -Force
    }
    New-Item ./suv_download_tmp -ItemType Directory
    Invoke-WebRequest -Uri "https://github.com/Microsoft/AirSim/releases/download/v1.2.0/car_assets.zip" -OutFile "./suv_download_tmp/car_assets.zip"
    if (Test-Path "./Unreal/Plugins/AirSim/Content/VehicleAdv/SUV") {
      Remove-Item "./Unreal/Plugins/AirSim/Content/VehicleAdv/SUV" -Recurse -Force
    }
    Expand-Archive -Path "./suv_download_tmp/car_assets.zip" -DestinationPath "./Unreal/Plugins/AirSim/Content/VehicleAdv"
    Remove-Item "./suv_download_tmp" -Recurse -Force
    
    if (-not (Test-Path "./Unreal/Plugins/AirSim/Content/VehicleAdv/SUV")) {
      Write-Warning "Unable to download high-polycount SUV. Your AirSim build will use the default vehicle."
    }
  } 
  else {
    "Not downloading high-poly car asset. The default unreal vehicle will be used." | Out-Host
  }
}

# install libraries
if ($isWindowsOS) {
  $compiler = "clang-cl"
}
else {
  $compiler = "clang"
}
$buildDir = "./build/build/$BuildMode"
cmake -S./cmake -B"$buildDir" -DCMAKE_INSTALL_PREFIX="$InstallPrefix" `
  -GNinja `
  -DCMAKE_C_COMPILER="$compiler" `
  -DCMAKE_CXX_COMPILER="$compiler" `
  -DBUILD_TESTS=ON `
  -DBUILD_EXAMPLES=ON `
  -DCMAKE_BUILD_TYPE="$BuildMode"

cmake --build $buildDir
cmake --install $buildDir