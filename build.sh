#! /bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

set -e

# update the rpclib git submodule
git submodule update --init --recursive 

#!/bin/bash
if [[ !(-f "/usr/bin/clang") || !(-f "/usr/bin/xclang++") ]]; then
	echo "ERROR: clang 3.9 is necessary to compile AirSim and run it in Unreal"
	echo "       please run : sudo apt-get install clang++-3.9"
	echo "       followed by: sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-3.9 60 --slave /usr/bin/clang++ clang++ /usr/bin/clang++-3.9 "

  echo "       We need to use clang because the Unreal Engine is built with clang as well and"
  echo "       there are some symbol inconsistencies in the C++ library with regard to C++11"
  echo "       (see GCC Dual ABI: # https://gcc.gnu.org/onlinedocs/libstdc++/manual/using_dual_abi.html)"

	exit 1
fi

export CC=/usr/bin/clang
export CXX=/usr/bin/clang++

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

build_dir=build_debug
echo "putting build in build_debug folder, to clean, just delete the directory..."

# this ensures the cmake files will be built in our $build_dir instead.
if [[ -f "../cmake/CMakeCache.txt" ]]; then
    rm "../cmake/CMakeCache.txt"
fi
if [[ -d "../cmake/CMakeFiles" ]]; then
    rm -rf "../cmake/CMakeFiles"
fi

if [[ ! -d $build_dir ]]; then
	mkdir -p $build_dir
	pushd $build_dir

	cmake ../cmake -DCMAKE_BUILD_TYPE=Debug \
		|| (cd .. && rm -r $build_dir && exit 1)
	popd &>/dev/null
fi

pushd $build_dir
# final linking of the binaries can fail due to a missing libc++abi library
# (happens on Fedora, see https://bugzilla.redhat.com/show_bug.cgi?id=1332306).
# So we only build the libraries here for now
make -j8 AirLib MavLinkCom
popd &>/dev/null


mkdir -p AirLib/lib/x64/Debug
mkdir -p AirLib/deps/rpclib
mkdir -p AirLib/deps/MavLinkCom
rsync -a --delete $build_dir/output/lib/ AirLib/lib/x64/Debug
rsync -a --delete external/rpclib/include AirLib/deps/rpclib
rsync -a --delete MavLinkCom/include AirLib/deps/MavLinkCom
rsync -a --delete AirLib Unreal/Plugins/AirSim/Source


echo ""
echo "============================================================"
echo "Now copy the Unreal/Plugins directory to the Unreal project:"
echo "rsync -t -r Unreal/plugins <unreal project_root>"
echo "  (<unreal project_root> contains a file named <project>.uproject)"
echo "============================================================"
echo "And do (required for building the Unreal plugin):"
echo "export EIGEN_ROOT=\"$EIGEN_ROOT\""

