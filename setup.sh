#! /bin/bash

set -x
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR" >/dev/null

#get sub modules
git submodule update --init --recursive

#TODO: below is probably not needed any longer
sudo apt-get install -y build-essential

# get clang, libc++
sudo rm -rf llvm-build
# for other OS, please see http://releases.llvm.org/download.html#3.9.1
wget http://releases.llvm.org/3.9.1/clang+llvm-3.9.1-x86_64-linux-gnu-ubuntu-16.04.tar.xz
mkdir -p llvm-build
tar -xf "clang+llvm-3.9.1-x86_64-linux-gnu-debian8.tar.xz" -C llvm-build
rm "clang+llvm-3.9.1-x86_64-linux-gnu-debian8.tar.xz"
mv ./llvm-build/clang* ./llvm-build/output

#install EIGEN library
sudo rm -rf ./AirLib/deps/eigen3/Eigen
echo "downloading eigen..."
wget http://bitbucket.org/eigen/eigen/get/3.3.2.zip
unzip 3.3.2.zip -d temp_eigen
mkdir -p AirLib/deps/eigen3
mv temp_eigen/eigen*/Eigen AirLib/deps/eigen3
rm -rf temp_eigen
rm 3.3.2.zip

popd >/dev/null

set +x
echo ""
echo "************************************"
echo "AirSim setup completed successfully!"
echo "************************************"
