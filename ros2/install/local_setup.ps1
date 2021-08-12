# generated from colcon_powershell/shell/template/prefix.ps1.em

# This script extends the environment with all packages contained in this
# prefix path.

# check environment variable for custom Python executable
if ($env:COLCON_PYTHON_EXECUTABLE) {
  if (!(Test-Path "$env:COLCON_PYTHON_EXECUTABLE" -PathType Leaf)) {
    echo "error: COLCON_PYTHON_EXECUTABLE '$env:COLCON_PYTHON_EXECUTABLE' doesn't exist"
    exit 1
  }
  $_colcon_python_executable="$env:COLCON_PYTHON_EXECUTABLE"
} else {
  # use the Python executable known at configure time
  $_colcon_python_executable="/usr/bin/python3"
  # if it doesn't exist try a fall back
  if (!(Test-Path "$_colcon_python_executable" -PathType Leaf)) {
    if (!(Get-Command "python3" -ErrorAction SilentlyContinue)) {
      echo "error: unable to find python3 executable"
      exit 1
    }
    $_colcon_python_executable="python3"
  }
}

# function to source another script with conditional trace output
# first argument: the path of the script
function _colcon_prefix_powershell_source_script {
  param (
    $_colcon_prefix_powershell_source_script_param
  )
  # source script with conditional trace output
  if (Test-Path $_colcon_prefix_powershell_source_script_param) {
    if ($env:COLCON_TRACE) {
      echo ". '$_colcon_prefix_powershell_source_script_param'"
    }
    . "$_colcon_prefix_powershell_source_script_param"
  } else {
    Write-Error "not found: '$_colcon_prefix_powershell_source_script_param'"
  }
}

# get all commands in topological order
$_colcon_ordered_commands = & "$_colcon_python_executable" "$(Split-Path $PSCommandPath -Parent)/_local_setup_util_ps1.py" ps1

# execute all commands in topological order
if ($env:COLCON_TRACE) {
  echo "Execute generated script:"
  echo "<<<"
  $_colcon_ordered_commands.Split([Environment]::NewLine, [StringSplitOptions]::RemoveEmptyEntries) | Write-Output
  echo ">>>"
}
$_colcon_ordered_commands.Split([Environment]::NewLine, [StringSplitOptions]::RemoveEmptyEntries) | Invoke-Expression
