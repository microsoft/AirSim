rm CMakeCache.txt 
rm -rf CMakeFiles
cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=gcc-6 -D CMAKE_CXX_COMPILER=g++-6 CMakeLists.txt
make
