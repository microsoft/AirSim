# Linux Build

## cmake

First you will need at least [cmake version  3.5](https://cmake.org/install/). 
If you don't have cmake version 3.* (for example, that is not the default on Ubuntu 14) you can run the following:

````
sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
sudo apt-get update
sudo apt-get install cmake
````

Next you need a version of [CLang compiler](http://releases.llvm.org/3.9.0/tools/clang/docs/ReleaseNotes.html) that supports `-std=c++14`.  Version 3.9 or newer should work.   The following commands will get you clang 3.9:
````
sudo apt-get update
wget -O - http://apt.llvm.org/llvm-snapshot.gpg.key | sudo apt-key add -
sudo apt-add-repository "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-3.9 main"
sudo apt-get update
sudo apt-get install clang-3.9 lldb-3.9
sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-3.9 60 --slave /usr/bin/clang++ clang++ /usr/bin/clang++-3.9
````

Next you will need the latest version of libc++ library, which you can build yourself by doing this:

````
# Checkout LLVM sources
git clone --depth=1 https://github.com/llvm-mirror/llvm.git llvm-source
git clone --depth=1 https://github.com/llvm-mirror/libcxx.git llvm-source/projects/libcxx
git clone --depth=1 https://github.com/llvm-mirror/libcxxabi.git llvm-source/projects/libcxxabi

export C_COMPILER=clang
export COMPILER=clang++

# Build and install libc++ (Use unstable ABI for better sanitizer coverage)
mkdir llvm-build && cd llvm-build
cmake -DCMAKE_C_COMPILER=${C_COMPILER} -DCMAKE_CXX_COMPILER=${COMPILER} \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=/usr \
      -DLIBCXX_ABI_UNSTABLE=ON \
      -DLLVM_USE_SANITIZER=${LIBCXX_SANITIZER} \
      ../llvm-source
make cxx -j2
sudo make install-cxxabi install-cxx
````

Now you can run the build.sh at the root level of the AirSim repo:

````
./build.sh
````
This will create a `build_debug` folder containing the build output and the cmake generated make files.

## Reset build

If for any reason you need to re-run cmake to regenerate new make files just deete the `build_debug` folder.
