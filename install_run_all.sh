#! /bin/bash

# get path of current script: https://stackoverflow.com/a/39340259/207661
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR"  >/dev/null

set -e
set -x

#confirm unreal install directory
UnrealDir=$1
if [[ !(-z "UnrealDir") ]]; then
	UnrealDir="$SCRIPT_DIR/UnrealEngine"
fi

read -p "Unreal will be installed in $UnrealDir. To change it invoke script with path argument. Continue? " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	popd >/dev/null
    exit 0
fi

#install unreal
if [[ !(-d "$UnrealDir") ]]; then
	git clone -b 4.17 https://github.com/EpicGames/UnrealEngine.git "$UnrealDir"
	pushd "$UnrealDir" >/dev/null

	./Setup.sh
	./GenerateProjectFiles.sh
	make

	popd >/dev/null
fi

#install airsim
./setup.sh
./build.sh

#start Unreal editor with Blocks project
pushd "$UnrealDir" >/dev/null
Engine/Binaries/Linux/UE4Editor "$SCRIPT_DIR/AirSim/Unreal/Environments/Blocks/Blocks.uproject" -game -log
popd >/dev/null

popd >/dev/null