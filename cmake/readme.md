# CMAKE

You need at least cmake version  3.4. 

## Windows

On Windows run this command to build debug bits:

    cmake -G "Visual Studio 14 2015 Win64" -D CMAKE_BUILD_TYPE=Debug CMakeLists.txt
    msbuild /p:Platform=x64 /p:Configuration=Debug AirSim.sln
    
To build release bits delete CMakeCache.txt and run this:
    
    cmake -G "Visual Studio 14 2015 Win64" -D CMAKE_BUILD_TYPE=Release CMakeLists.txt
    msbuild /p:Platform=x64 /p:Configuration=Release AirSim.sln

## Linux    

If you don't cmake version 3.*  you can run the following:
````
sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
sudo apt-get update
````

On Linux you first need a version of GCC that supports `-std=c++14`.  Version 4.9 or newer should work.  
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

Now if gcc version 4.9 is not your default version of gcc, you can tell cmake to switch to that version by
adding the following additional cmake command line arguments:

````
-D CMAKE_C_COMPILER=gcc-4.9 -D CMAKE_CXX_COMPILER=g++-4.9 
````
    
