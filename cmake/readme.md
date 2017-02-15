On Windows run this command to build debug bits:

    cmake -G "Visual Studio 14 2015 Win64" -D CMAKE_BUILD_TYPE=Debug CMakeLists.txt
    msbuild /p:Platform=x64 /p:Configuration=Debug AirSim.sln
    
To build release bits delete CMakeCache.txt and run this:
    
    cmake -G "Visual Studio 14 2015 Win64" -D CMAKE_BUILD_TYPE=Release CMakeLists.txt
    msbuild /p:Platform=x64 /p:Configuration=Release AirSim.sln
    
    