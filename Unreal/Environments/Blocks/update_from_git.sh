#! /bin/bash

# get path of current script: https://stackoverflow.com/a/39340259/207661
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR"

set -e
set +x

rsync -a  --exclude 'temp' --delete ../../Plugins/AirSim Plugins/AirSim
rsync -a  --exclude 'temp' --delete ../../../AirLib Plugins/AirSim/Source/AirLib

popd