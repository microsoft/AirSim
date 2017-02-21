# Linux Build

## cmake    

First you will need at least [cmake version  3.4](https://cmake.org/install/).  
If you don't have cmake version 3.* (for example, that is not the default on Ubuntu 14) you can run the following:

````
sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
sudo apt-get update
````

Then get the right version of Eigen, see [Install Eigen](install_eigen.md).  
You also need the right version of Boost, see [Install Boost](install_boost.md).

Next you need a version of GCC that supports `-std=c++14`.  Version 6, or newer should work.  
If you don't have version 6 you can get it by running these commands:
````
sudo apt-get update
sudo apt-get install build-essential software-properties-common -y
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
sudo apt-get update
sudo apt-get install gcc-snapshot -y
sudo apt-get update
sudo apt-get install gcc-6 g++-6 -y
````

Now you can either do this, to make gcc 6 your default gcc compiler:
````
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 60 --slave /usr/bin/g++ g++ /usr/bin/g++-6
````
or you can add the following to the cmake command line instead:
````
-D CMAKE_C_COMPILER=gcc-6 -D CMAKE_CXX_COMPILER=g++-6
````

Then run cmake:
````
cmake -D CMAKE_BUILD_TYPE=Debug CMakeLists.txt

````

Now you are ready to build:
````
make
````

## Reset cmake

Just a tid bit for those not familiar with cmake, if for any reason you need to re-run cmake to regenerate new make files
(perhaps you want to move boost or eigen) then the following is the equivalent of `clean` for cmake:
````
rm CMakeCache.txt 
rm -rf CMakeFiles
````


## Windows cmake

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

