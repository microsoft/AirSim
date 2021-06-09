#! /bin/bash

# get path of current script: https://stackoverflow.com/a/39340259/207661
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR"  >/dev/null

set -e
set -x

# Get Unreal install directory
UnrealDir=$1
if [[ !(-z "UnrealDir") ]]; then
    UnrealDir="$SCRIPT_DIR/UnrealEngine"
fi

# Install Unreal Engine
./install_unreal.sh $1

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
