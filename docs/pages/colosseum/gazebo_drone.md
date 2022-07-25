# Welcome to GazeboDrone

GazeboDrone allows connecting a gazebo drone to the AirSim drone, using the gazebo drone as a flight dynamic model (FDM) and AirSim to generate environmental sensor data. It can be used for **Multicopters**, **Fixed-wings** or any other vehicle.


## Dependencies

### Gazebo

Make sure you have installed gazebo dependencies:

```
sudo apt-get install libgazebo9-dev
```

### AirLib

This project is built with GCC 8, so AirLib needs to be built with GCC 8 too. 
Run from your AirSim root folder:  
```
./clean.sh
./setup.sh
./build.sh --gcc
```

## AirSim simulator

The AirSim UE plugin needs to be built with clang, so you can't use the one compiled in the previous step. You can use [our binaries](https://github.com/microsoft/AirSim/releases) or you can clone AirSim again in another folder and buid it without the above option, then you can [run Blocks](build_linux.md#how-to-use-airsim) or your own environment.


### AirSim settings

Inside your `settings.json` file you need to add this line:  
`"PhysicsEngineName":"ExternalPhysicsEngine"`.  
You may want to change the visual model of the AirSim drone, for that you can follow [this tutorial.](https://youtu.be/Bp86WiLUC80)


## Build 

Execute this from your AirSim root folder:  
```
cd GazeboDrone
mkdir build && cd build
cmake -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8 ..
make
```

## Run

First run the AirSim simulator and your Gazebo model and then execute this from your AirSim root folder:

```
cd GazeboDrone/build
./GazeboDrone
```

