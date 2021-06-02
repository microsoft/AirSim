# General

Derived from http://gazebosim.org/tutorials?tut=topics_subscribed

Make sure you have installed gazebo dependencies

```
sudo apt-get install libgazebo9-dev
```

# AirLib build

This project is built with g++, so AirLib needs to be built with g++ too. Change lines 56 and 57 of AirSim/build.sh with:
```
export CC="gcc-8"
export CXX="g++-8"
```
then run ./setup.sh and ./build.sh

# AirSim plugin

The AirSim UE plugin needs to be built with clang, so you can't use the one compiled in the previous step. You can use [our binaries](https://github.com/microsoft/AirSim/releases) or you can clone AirSim again in another folder without the above change, and with that one, [run Blocks](https://microsoft.github.io/AirSim/build_linux/#how-to-use-airsim) or your environment.


# AirSim settings

Inside your `settings.json` file add this line:  
`"PhysicsEngineName":"ExternalPhysicsEngine"`
# Build

```
mkdir build && cd build
cmake ..
make
```

# Run

Run AirSim and Gazebo

then run:

```
cd build
./GazeboDrone
```

