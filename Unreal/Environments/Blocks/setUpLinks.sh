#! /bin/bash

# Establish symbolic links to point from Unreal project to AirSim plugin code

./clean.sh

mkdir Plugins

cd Plugins
ln -s ../../../Plugins/AirSim
cd AirSim/Source
ln -s ../../../../AirLib
cd ../../..

