# Linux Build

The Linux build system is in progress... please stay tuned...

## cmake    

First you will need at least [cmake version  3.4](https://cmake.org/install/).  
If you don't have cmake version 3.* (for example, that is not the default on Ubuntu 14) you can run the following:

````
sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
sudo apt-get update
````

On Linux you first need a version of GCC that supports `-std=c++14`.  Version 4.9, or newer should work.  
If you don't have version 4.9 you can get it by running these commands:
````
sudo apt-get install build-essential
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-4.9 g++-4.9 cpp-4.9
````

Then run this:
````
cmake -D CMAKE_BUILD_TYPE=Debug CMakeLists.txt
make
````

Now if gcc version 4.9 is not your default version of gcc, you will also need to tell cmake to use this version by
adding the following additional cmake command line arguments:

````
-D CMAKE_C_COMPILER=gcc-4.9 -D CMAKE_CXX_COMPILER=g++-4.9 
````

Now type `make`.  

STATUS: all of the MavLinkCom library builds and runs on Linux, but the AirLib code is running into a weird
difference in how GCC implements some -std=c++14 features... stay tuned...

## Windows

You can also use cmake on Windows, but we have already checked in a .sln and .vcxproj files that are nicer than what
cmake creates.  This is why we have separated the CMakeLists.txt files into a different directory, this ensures cmake 
doesn't override our hand built VS project files.

This will create the debug build:

    cmake -G "Visual Studio 14 2015 Win64" -D CMAKE_BUILD_TYPE=Debug CMakeLists.txt
    msbuild /p:Platform=x64 /p:Configuration=Debug AirSim.sln
    
To build release bits you have to delete CMakeCache.txt (it's not clear that cmake supports building
one set of vcxproj make files that can do both debug and release, if someone knows how please submit a pull request!)
and run this:
    
    cmake -G "Visual Studio 14 2015 Win64" -D CMAKE_BUILD_TYPE=Release CMakeLists.txt
    msbuild /p:Platform=x64 /p:Configuration=Release AirSim.sln

