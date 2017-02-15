#Windows

On Windows run this command to build debug bits:

    cmake -G "Visual Studio 14 2015 Win64" -D CMAKE_BUILD_TYPE=Debug CMakeLists.txt
    msbuild /p:Platform=x64 /p:Configuration=Debug AirSim.sln
    
To build release bits delete CMakeCache.txt and run this:
    
    cmake -G "Visual Studio 14 2015 Win64" -D CMAKE_BUILD_TYPE=Release CMakeLists.txt
    msbuild /p:Platform=x64 /p:Configuration=Release AirSim.sln

#Linux    
On Linux you first need a version of GCC that supports `-std=c++14`.  Version 4.9 or newer should work.  Then run this:

    cmake -D CMAKE_C_COMPILER=gcc-4.9 -D CMAKE_CXX_COMPILER=g++-4.9 -D CMAKE_BUILD_TYPE=Debug CMakeLists.txt
    make
    
