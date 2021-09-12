#! /bin/bash

# get path of current script: https://stackoverflow.com/a/39340259/207661
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR"  >/dev/null

set -e
set -x

debug=false
gcc=false
# Parse command line arguments
while [[ $# -gt 0 ]]
do
    key="$1"

    case $key in
    --debug)
        debug=true
        shift # past argument
        ;;
    --gcc)
        gcc=true
        shift # past argument
        ;;
    esac

done

function version_less_than_equal_to() { test "$(printf '%s\n' "$@" | sort -V | head -n 1)" = "$1"; }

# check for local cmake build created by setup.sh
if [ -d "./cmake_build" ]; then
    if [ "$(uname)" == "Darwin" ]; then
        CMAKE="$(greadlink -f cmake_build/bin/cmake)"
    else
        CMAKE="$(readlink -f cmake_build/bin/cmake)"
    fi
else
    CMAKE=$(which cmake)
fi

if $debug; then
    buildType="Debug"
else
    buildType="Release"
fi

# variable for build output
build_dir=./build/build/$buildType
install_dir=./build/install/$buildType

if [ "$(uname)" == "Darwin" ]; then
    export CC=/usr/local/opt/llvm@8/bin/clang
    export CXX=/usr/local/opt/llvm@8/bin/clang++
else
    if $gcc; then
        export CC="gcc-8"
        export CXX="g++-8"
    else
        export CC="clang-8"
        export CXX="clang++-8"
    fi
fi

echo "putting build in $build_dir folder, to clean, just delete the directory..."

# this ensures the cmake files will be built in our $build_dir instead.
if [[ -f "./cmake/CMakeCache.txt" ]]; then
    rm "./cmake/CMakeCache.txt"
fi
if [[ -d "./cmake/CMakeFiles" ]]; then
    rm -rf "./cmake/CMakeFiles"
fi

"$CMAKE" -S./cmake -B$build_dir -DCMAKE_BUILD_TYPE=$buildType -DCMAKE_INSTALL_PREFIX=$install_dir -DFORCE_INSTALL_3RDPARTY=ON \
        || (rm -r $build_dir && exit 1) 

# final linking of the binaries can fail due to a missing libc++abi library
# (happens on Fedora, see https://bugzilla.redhat.com/show_bug.cgi?id=1332306).
# So we only build the libraries here for now
"$CMAKE" --build $build_dir -j`nproc` --config $buildType

"$CMAKE" --install $build_dir  --config $buildType

# Update AirLib with new binaries
mkdir -p ./Unreal/Plugins/AirSim/Source/AirLib/$buildType
rsync -a --delete $install_dir/ ./Unreal/Plugins/AirSim/Source/AirLib/$buildType/ --exclude=bin --exclude=share --exclude=cmake

set +x

echo ""
echo ""
echo "=================================================================="
echo " AirSim plugin is built! Here's how to build Unreal project."
echo "=================================================================="
echo "If you are using Blocks environment, its already updated."
echo "If you are using your own environment, update plugin using,"
echo "rsync -a --delete Unreal/Plugins path/to/MyUnrealProject"
echo ""
echo "For help see:"
echo "https://github.com/Microsoft/AirSim/blob/master/docs/build_linux.md"
echo "=================================================================="

popd >/dev/null
