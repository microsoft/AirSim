#! /bin/bash

# get path of current script: https://stackoverflow.com/a/39340259/207661
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR"  >/dev/null

set -e
set -x

#confirm unreal install directory
UnrealDir=$1
if [[ -z "$UnrealDir" ]]; then
    # UnrealDir variable must be set like '/Users/Shared/Epic\ Games/UE_4.16'
    echo "UnrealDir is not set."
    exit 1
fi

#install airsim
./setup.sh
./build.sh

#start Unreal editor with Blocks project
pushd "$UnrealDir" >/dev/null
if [ "$(uname)" == "Darwin" ]; then
    Engine/Binaries/Mac/UE4Editor.app/Contents/MacOS/UE4Editor "$SCRIPT_DIR/Unreal/Environments/Blocks/Blocks.uproject" -game -log
else
    Engine/Binaries/Linux/UE4Editor "$SCRIPT_DIR/Unreal/Environments/Blocks/Blocks.uproject" -game -log
fi
popd >/dev/null

popd >/dev/null
