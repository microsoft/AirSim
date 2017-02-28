# Get the location of this script, it is assumed you already cloned "main" and you are running the
# script from that repo.  It will then put everything else in a ROS workspace at "../catkin_ws"
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
pushd $SCRIPTPATH

# update the rpclib git submodule
pushd ..
git submodule update --init --recursive 
popd

if [ -f "CMakeCache.txt" ]; then
rm CMakeCache.txt 
fi

if [ -f "CMakeFiles" ]; then
rm -rf CMakeFiles
fi

GCCARGS="-D CMAKE_BUILD_TYPE=Debug"

GCCVERSION=$(gcc -v 2>&1 | sed -n "/^gcc version/p" | sed -e "s/^gcc version \([0-9]\)/\1/g" | cut -d"." -f1)
if [ $GCCVERSION -lt 7 ]; then
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