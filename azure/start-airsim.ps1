# Script parameters
$airSimExecutable = "c:\AirSim\Blocks\blocks.exe"
$airSimProcessName = "Blocks"

# Ensure proper path
$env:Path = 
    [System.Environment]::GetEnvironmentVariable("Path","Machine") + ";" +
    [System.Environment]::GetEnvironmentVariable("Path","User")

# Install python app requirements
pip3 install -r .\app\requirements.txt

# Overwrite AirSim configuration
New-Item -ItemType Directory -Force -Path $env:USERPROFILE\Documents\AirSim\
copy .\app\settings.json $env:USERPROFILE\Documents\AirSim\

# Kill previous AirSim instance
Stop-Process -Name $airSimProcessName -Force -ErrorAction SilentlyContinue
sleep 2

# Start new AirSim instance
Start-Process -NoNewWindow  -FilePath $airSimExecutable -ArgumentList "-RenderOffScreen"
echo "Starting the AirSim environment has completed."
