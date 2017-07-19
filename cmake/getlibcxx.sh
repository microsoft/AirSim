#! /bin/bash

set +x
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR"

# add llvm-source to root of AirSim
cd ..

# Checkout LLVM sources
if [[ ! -d "llvm-source" ]]; then 
	git clone --depth=1 -b release_39  https://github.com/llvm-mirror/llvm.git llvm-source
	git clone --depth=1 -b release_39  https://github.com/llvm-mirror/libcxx.git llvm-source/projects/libcxx
	git clone --depth=1 -b release_39  https://github.com/llvm-mirror/libcxxabi.git llvm-source/projects/libcxxabi
else
	echo "folder llvm-source already exists, skipping git clone..."
fi

# Build and install libc++ 
rm -rf llvm-build
mkdir llvm-build && cd llvm-build

export C_COMPILER=clang
export COMPILER=clang++

# Build and install libc++ 
cmake -DCMAKE_C_COMPILER=${C_COMPILER} -DCMAKE_CXX_COMPILER=${COMPILER} \
      -DLIBCXX_INSTALL_EXPERIMENTAL_LIBRARY=ON  \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=./output \
            ../llvm-source

make cxx

sudo make install-libcxx install-libcxxabi 

popd