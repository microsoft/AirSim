#! /bin/bash

set -x
set -e

if [ "$(uname)" == "Darwin" ]; then
    export CC=/usr/local/opt/llvm@8/bin/clang
    export CXX=/usr/local/opt/llvm@8/bin/clang++
else
    export CC="clang-8"
    export CXX="clang++-8"
fi

# check for local cmake build created by setup.sh
if [ -d "../cmake_build" ]; then
    if [ "$(uname)" = "Darwin" ]; then
        CMAKE="$(greadlink -f ../cmake_build/bin/cmake)"
    else
        CMAKE="$(readlink -f ../cmake_build/bin/cmake)"
    fi
else
    CMAKE=$(which cmake)
fi

"$CMAKE"  \
 -S./AirLibWrapper/AirsimWrapper \
 -B./build/build \
 -DCMAKE_INSTALL_LIBDIR=. \
 -DCMAKE_INSTALL_PREFIX=./UnityDemo/Assets/Plugins \
 -DCMAKE_BUILD_TYPE=Release
"$CMAKE" --build  ./build/build --config Release --target AirsimWrapper
"$CMAKE" --install ./build/build --config Release

rm UnityDemo/Assets/Plugins/*.a
rm -r UnityDemo/Assets/Plugins/lib
rm -r UnityDemo/Assets/Plugins/include
rm -r UnityDemo/Assets/Plugins/share