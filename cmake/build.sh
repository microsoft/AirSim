# This script is for building AirSim with GCC-6
# You should NOT use this with Unreal.
# If you want bits that work in Unreal you need to use ../build.sh
# For pre-requisite tools needed to run this script see
# https://github.com/Microsoft/AirSim/blob/master/docs/build_linux.md

# First get the location of this script, it is assumed you already cloned "main" and you are running the
# script from that repo.  It will then put everything else in a ROS workspace at "../catkin_ws"
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
pushd $SCRIPTPATH

if [[ ! -d "$EIGEN_ROOT" ]]; then
        if [[ ! -d eigen ]]; then
                echo "downloading eigen..."
                wget http://bitbucket.org/eigen/eigen/get/3.3.2.zip
                unzip 3.3.2.zip -d eigen
                pushd eigen
                mv eigen* eigen3
                echo "3.3.2" > version
                popd &>/dev/null
                rm 3.3.2.zip
        fi
        export EIGEN_ROOT="$(pwd)/eigen"
fi
popd

if [ -f "CMakeCache.txt" ]; then
rm CMakeCache.txt
fi

if [ -d "CMakeFiles" ]; then
rm -rf CMakeFiles
fi

GCCARGS="-D CMAKE_BUILD_TYPE=Debug"

GCCVERSION=$(gcc -v 2>&1 | sed -n "/^gcc version/p" | sed -e "s/^gcc version \([0-9]\)/\1/g" | cut -d"." -f1)
if [ $GCCVERSION -lt 5 ]; then
    GCCARGS="$GCCARGS -D CMAKE_C_COMPILER=gcc-6 -D CMAKE_CXX_COMPILER=g++-6"
fi
cmake $GCCARGS CMakeLists.txt

make

pushd ..
mkdir -p AirLib/lib/x64/Debug
mkdir -p AirLib/deps/rpclib
mkdir -p AirLib/deps/MavLinkCom
rsync -a --delete cmake/output/lib/ AirLib/lib/x64/Debug
rsync -a --delete external/rpclib/include AirLib/deps/rpclib
rsync -a --delete MavLinkCom/include AirLib/deps/MavLinkCom
rsync -a --delete AirLib Unreal/Plugins/AirSim/Source
popd
