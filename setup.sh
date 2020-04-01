#! /bin/bash
set -x

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR" >/dev/null

downloadHighPolySuv=true
gccBuild=false
MIN_CMAKE_VERSION=3.10.0
MIN_GCC_VERSION=6.0.0
function version_less_than_equal_to() { test "$(printf '%s\n' "$@" | sort -V | head -n 1)" = "$1"; }

# Parse command line arguments
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    --no-full-poly-car)
    downloadHighPolySuv=false
    shift # past value
    ;;
    --gcc)
    gccBuild=true
    shift # past argument
    ;;
esac
done

if $gccBuild; then
    # gcc tools
    if ! which gcc; then
        # GCC not installed
        gcc_ver=0
    else
        gcc_ver=$(gcc -dumpfullversion)
    fi

    if version_less_than_equal_to $gcc_ver $MIN_GCC_VERSION; then
        if [ "$(uname)" == "Darwin" ]; then # osx
            brew update
            brew install gcc-6 g++-6
        else
            sudo add-apt-repository ppa:ubuntu-toolchain-r/test
            sudo apt-get -y update
            sudo apt-get install -y gcc-6 g++-6
        fi
    else
        echo "Already have good version of gcc: $gcc_ver"
    fi
else
    # llvm tools
    if [ "$(uname)" == "Darwin" ]; then # osx
        brew update
        brew tap llvm-hs/homebrew-llvm
        brew install llvm@8
    else #linux
        #install clang and build tools
        VERSION=$(lsb_release -rs | cut -d. -f1)
        # Since Ubuntu 17 clang is part of the core repository
        # See https://packages.ubuntu.com/search?keywords=clang-8
        if [ "$VERSION" -lt "17" ]; then
            wget -O - http://apt.llvm.org/llvm-snapshot.gpg.key | sudo apt-key add -
            sudo apt-get update
        fi
        sudo apt-get install -y clang-8 clang++-8 libc++-8-dev libc++abi-8-dev
    fi
fi

#give user perms to access USB port - this is not needed if not using PX4 HIL
#TODO: figure out how to do below in travis
if [ "$(uname)" == "Darwin" ]; then # osx
    if [[ ! -z "${whoami}" ]]; then #this happens when running in travis
        sudo dseditgroup -o edit -a `whoami` -t user dialout
    fi

    brew install wget
    brew install coreutils
    brew install cmake  # should get cmake 3.8

else #linux
    if [[ ! -z "${whoami}" ]]; then #this happens when running in travis
        sudo /usr/sbin/useradd -G dialout $USER
        sudo usermod -a -G dialout $USER
    fi

    #install additional tools
    sudo apt-get install -y build-essential
    sudo apt-get install -y unzip
fi

if ! which cmake; then
    # CMake not installed
    cmake_ver=0
else
    cmake_ver=$(cmake --version 2>&1 | head -n1 | cut -d ' ' -f3 | awk '{print $NF}')
fi

#download cmake - v3.10.2 is not out of box in Ubuntu 16.04
if version_less_than_equal_to $cmake_ver $MIN_CMAKE_VERSION; then
    if [[ ! -d "cmake_build/bin" ]]; then
        echo "Downloading cmake..."
        wget https://cmake.org/files/v3.10/cmake-3.10.2.tar.gz \
            -O cmake.tar.gz
        tar -xzf cmake.tar.gz
        rm cmake.tar.gz
        rm -rf ./cmake_build
        mv ./cmake-3.10.2 ./cmake_build
        pushd cmake_build
        ./bootstrap
        make
        popd
    fi
else
    echo "Already have good version of cmake: $cmake_ver"
fi

# Download rpclib
if [ ! -d "external/rpclib/rpclib-2.2.1" ]; then
    echo "*********************************************************************************************"
    echo "Downloading rpclib..."
    echo "*********************************************************************************************"

    wget  https://github.com/rpclib/rpclib/archive/v2.2.1.zip

    # remove previous versions
    rm -rf "external/rpclib"

    mkdir -p "external/rpclib"
    unzip v2.2.1.zip -d external/rpclib
    rm v2.2.1.zip
fi

# Download high-polycount SUV model
if $downloadHighPolySuv; then
    if [ ! -d "Unreal/Plugins/AirSim/Content/VehicleAdv" ]; then
        mkdir -p "Unreal/Plugins/AirSim/Content/VehicleAdv"
    fi
    if [ ! -d "Unreal/Plugins/AirSim/Content/VehicleAdv/SUV/v1.2.0" ]; then
            echo "*********************************************************************************************"
            echo "Downloading high-poly car assets.... The download is ~37MB and can take some time."
            echo "To install without this assets, re-run setup.sh with the argument --no-full-poly-car"
            echo "*********************************************************************************************"

            if [ -d "suv_download_tmp" ]; then
                rm -rf "suv_download_tmp"
            fi
            mkdir -p "suv_download_tmp"
            cd suv_download_tmp
            wget  https://github.com/Microsoft/AirSim/releases/download/v1.2.0/car_assets.zip
            if [ -d "../Unreal/Plugins/AirSim/Content/VehicleAdv/SUV" ]; then
                rm -rf "../Unreal/Plugins/AirSim/Content/VehicleAdv/SUV"
            fi
            unzip car_assets.zip -d ../Unreal/Plugins/AirSim/Content/VehicleAdv
            cd ..
            rm -rf "suv_download_tmp"
    fi
else
    echo "### Not downloading high-poly car asset (--no-full-poly-car). The default unreal vehicle will be used."
fi

echo "Installing EIGEN library..."

rm -rf ./AirLib/deps/eigen3/Eigen
echo "downloading eigen..."
wget -O eigen3.zip https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip
unzip eigen3.zip -d temp_eigen
mkdir -p AirLib/deps/eigen3
mv temp_eigen/eigen*/Eigen AirLib/deps/eigen3
rm -rf temp_eigen
rm eigen3.zip

popd >/dev/null

set +x
echo ""
echo "************************************"
echo "AirSim setup completed successfully!"
echo "************************************"
