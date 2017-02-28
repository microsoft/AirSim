#! /bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

set -e

# we need to use clang because the Unreal Engine is built with clang as well and
# there are some symbol inconsistencies in the C++ library with regard to C++11
# (see GCC Dual ABI: # https://gcc.gnu.org/onlinedocs/libstdc++/manual/using_dual_abi.html)

#!/bin/bash
if [[ !(-f "/usr/bin/clang") || !(-f "/usr/bin/clang++") ]]; then
	echo "clang is necessary to compile AirSim"
fi

export CC=/usr/bin/clang
export CXX=/usr/bin/clang++


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


boost_dir="$(pwd)/boost/boost_1_63_0"
# get & build boost
if [[ ! -d boost ]]; then
	ldconfig -p | grep -q libc++
	if [ $? -ne 0 ]; then
		echo "it's necessary libc++ to compile boost"
		exit 1
	fi

	# because we are using Clang, we cannot use the system's boost libs, because
	# we could run into the same ABI problems as stated above
	echo "downloading & building boost..."
	wget https://sourceforge.net/projects/boost/files/boost/1.63.0/boost_1_63_0.zip/download
	mkdir boost
	unzip download -d boost
	mkdir "$boost_dir/installation"

	pushd "$boost_dir"
	./bootstrap.sh --prefix="$boost_dir/installation" --without-libraries=python
	./b2 -j8 toolset=clang cxxflags="-fPIC -stdlib=libc++" linkflags="-stdlib=libc++" \
		runtime-link=shared variant=release link=static threading=multi install
	rm ../../download

	popd &>/dev/null
fi

export BOOST_ROOT=$boost_dir/installation


build_dir=build_debug
# to clean, just delete the directory...


if [[ ! -d $build_dir ]]; then
	mkdir -p $build_dir
	pushd $build_dir
	cmake ../cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_FIND_ROOT_PATH="$BOOST_ROOT" \
		|| (cd .. && rm -r $build_dir && exit 1)
	popd &>/dev/null
fi

pushd $build_dir
# final linking of the binaries can fail due to a missing libc++abi library
# (happens on Fedora, see https://bugzilla.redhat.com/show_bug.cgi?id=1332306).
# So we only build the libraries here for now
make -j8 AirLib MavLinkCom
popd &>/dev/null


cp -p AirLib/deps/rpclib/lib/x64/Debug/librpc.a AirLib/deps/rpclib/lib
cp -p $build_dir/AirLib/lib/libAirLib.a AirLib/lib
cp -rp MavLinkCom/include AirLib/deps/MavLinkCom
cp -rp $build_dir/MavLinkCom/lib AirLib/deps/MavLinkCom
cp -rp AirLib Unreal/Plugins/AirSim/Source
rm -rf Unreal/Plugins/AirSim/Source/AirLib/deps/rpclib/rpclib

echo ""
echo "============================================================"
echo "Now copy the Unreal/Plugins directory to the Unreal project:"
echo "rsync -t -r Unreal/plugins <unreal project_root>"
echo "  (<unreal project_root> contains a file named <project>.uproject)"
echo "============================================================"
echo "And do (required for building the Unreal plugin):"
echo "export BOOST_ROOT=\"$BOOST_ROOT\""

