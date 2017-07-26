#! /bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR"

# add llvm-source to root of AirSim
cd ..

rm -rf  llvm-source
rm -rf  llvm-build

popd