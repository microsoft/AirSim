#! /bin/bash

# get path of current script: https://stackoverflow.com/a/39340259/207661
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR" >/dev/null

set -e
set -x

./clean.sh

rsync -a  --exclude 'temp' --delete ../../Plugins/AirSim Plugins/AirSim
rsync -a  --exclude 'temp' --delete ../../../AirLib Plugins/AirSim/Source/AirLib

popd >/dev/null