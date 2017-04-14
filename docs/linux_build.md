# Linux Build

These are the instructions to build AirSim on a Linux machine.
Note: you can also do this from [BashOnWindows](https://msdn.microsoft.com/en-us/commandline/wsl/install_guide)
but make sure you are `not` using a `Visual Studio Command Prompt` because we don't want cmake to accodemtally find VC++ and try and use that.

We need to use `clang compiler` because Unreal engine requires that.  

## cmake

First you will need at least [cmake version  3.5](https://cmake.org/install/). 
If you don't have cmake version 3.5 (for example, 3.2.2 is the default on Ubuntu 14) you can run the following:

````
mkdir ~/cmake-3.5.1
cd ~/cmake-3.5.1
wget https://cmake.org/files/v3.5/cmake-3.5.1-Linux-x86_64.sh
sh cmake-3.5.1-Linux-x86_64.sh --prefix ~/cmake-3.5.1
# Answer 'n' to the question about creating another cmake-3.5.1-Linux-x86_64 folder
sudo update-alternatives --install /usr/bin/cmake cmake ~/cmake-3.5.1/bin/cmake 60
````

Now type `cmake --version` to make sure your cmake version is 3.5.1.

Next you need a version of [CLang compiler](http://releases.llvm.org/3.9.0/tools/clang/docs/ReleaseNotes.html) that supports `-std=c++14`.  Version 3.9 or newer should work.  To install it you first need the archive signature:
````
wget -O - http://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add -
````

If you are using Ubuntu 16.04 (Xenial) then you can simply do this:
````
sudo apt-add-repository "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-3.9 main"
sudo apt-get update
````

For other flavors of Linux, you can get more instructions from [http://apt.llvm.org/](http://apt.llvm.org/).

Then run the following:

````
sudo apt-get update
sudo apt-get install clang-3.9 clang++-3.9
````

Note: On Ubuntu 16.04 you may be missing libjsoncpp0 which is no longer available in the package manager, you can [download the relevant dpkg here](http://packages.ubuntu.com/trusty/amd64/libjsoncpp0/download) and install with `sudo dpkg -i libjsoncpp0_0.6.0~*.deb`.

More detailed instructions are available here: [http://apt.llvm.org/](http://apt.llvm.org/).
Now make clang-3.9 your default version of clang with this command:

````
sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-3.9 60 --slave /usr/bin/clang++ clang++ /usr/bin/clang++-3.9
````
Next you will need the latest version of libc++ library, which you can get by running this:

````
./cmake/getlibcxx.sh
````

Now you can run the build.sh at the root level of the AirSim repo:

````
./build.sh
````
This will create a `build_debug` folder containing the build output and the cmake generated make files.

## Reset build

If for any reason you need to re-run cmake to regenerate new make files just deete the `build_debug` folder.

## Running Unreal on Linux

Now you are ready to follow these instructions to get [Unreal working on Linux](https://wiki.unrealengine.com/Building_On_Linux#Clang) but note that everywhere
you see Clang3.5 in the Unreal documentation replace that with clang3.9.
