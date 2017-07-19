#! /bin/bash

set +x
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR"

#get sub modules
git submodule update --init --recursive

#get clang 3.9
sudo apt-get install -y build-essential
wget -O - http://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add -
sudo apt-get update
sudo apt-get install -y clang-3.9 clang++-3.9
#other packages
#sudo apt-get install -y clang-3.9-doc libclang-common-3.9-dev libclang-3.9-dev libclang1-3.9 libclang1-3.9-dbg libllvm-3.9-ocaml-dev libllvm3.9 libllvm3.9-dbg lldb-3.9 llvm-3.9 llvm-3.9-dev llvm-3.9-doc llvm-3.9-examples llvm-3.9-runtime clang-format-3.9 python-clang-3.9 libfuzzer-3.9-dev

#get libc++ source
if [[ ! -d "llvm-source" ]]; then 
	git clone --depth=1 -b release_39  https://github.com/llvm-mirror/llvm.git llvm-source
	git clone --depth=1 -b release_39  https://github.com/llvm-mirror/libcxx.git llvm-source/projects/libcxx
	git clone --depth=1 -b release_39  https://github.com/llvm-mirror/libcxxabi.git llvm-source/projects/libcxxabi
else
	echo "folder llvm-source already exists, skipping git clone..."
fi

#build libc++
rm -rf llvm-build
mkdir llvm-build && cd llvm-build

export C_COMPILER=clang-3.9
export COMPILER=clang++-3.9

cmake -DCMAKE_C_COMPILER=${C_COMPILER} -DCMAKE_CXX_COMPILER=${COMPILER} \
      -DLIBCXX_INSTALL_EXPERIMENTAL_LIBRARY=ON  \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=./output \
            ../llvm-source

make cxx

#install libc++ locally in output folder
sudo make install-libcxx install-libcxxabi 

#install EIGEN library
if [[ -z "${EIGEN_ROOT}" ]]; then 
	echo "EIGEN_ROOT variable is not set"
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
fi

popd


