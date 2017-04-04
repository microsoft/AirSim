# Linux Build

These are the instructions to build AirSim on a Linux machine.
Note: you can also do this from [BashOnWindows](https://msdn.microsoft.com/en-us/commandline/wsl/install_guide)
but make sure you are `not` using a `Visual Studio Command Prompt` because we don't want cmake to accodemtally find VC++ and try and use that.

We need to use `clang compiler` because Unreal engine requires that.  

## cmake

First you will need at least [cmake version  3.5](https://cmake.org/install/). 
If you don't have cmake version 3.* (for example, that is not the default on Ubuntu 14) you can run the following:

````
sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
sudo apt-get update
sudo apt-get install cmake
````

If this doesn't work you can install it manually using these commands:

````
wget https://cmake.org/files/v3.5/cmake-3.5.1-Linux-x86_64.sh
sh cmake-3.5.1-Linux-x86_64.sh .
````

Next you need a version of [CLang compiler](http://releases.llvm.org/3.9.0/tools/clang/docs/ReleaseNotes.html) that supports `-std=c++14`.  Version 3.9 or newer should work.   The following commands will get you clang 3.9:
````
sudo apt-get update
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install clang-3.9 clang++-3.9
````

If this doesn't work please follow detailed instructions on [http://apt.llvm.org/](http://apt.llvm.org/).

````
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