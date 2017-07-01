# Linux Build

These are the instructions to build AirSim on **Ubuntu 16.04 LTS**. Theoratically you can build on other distros and versions as well but we haven't tested it (and 
mostly likely there are extra steps).

## Install Linux Pre-requisites
### cmake
First you will need at least [cmake version  3.5](https://cmake.org/install/). If you are using Ubuntu 16.04 LTS, you likely already have it. You can check this by command `cmake --version`.
If you don't have it then follow [these instructions](cmake.md).

### Clang compiler
Unreal engine requires clang compiler 3.9 or higher. To install it use following commands. For other flavors of Linux and more info, please see [http://apt.llvm.org/](http://apt.llvm.org/).
````
sudo apt-get install build-essential
wget -O - http://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add -
sudo apt-get update
sudo apt-get install clang-3.9 clang++-3.9 clang-3.9-doc libclang-common-3.9-dev libclang-3.9-dev libclang1-3.9 libclang1-3.9-dbg libllvm-3.9-ocaml-dev libllvm3.9 libllvm3.9-dbg lldb-3.9 llvm-3.9 llvm-3.9-dev llvm-3.9-doc llvm-3.9-examples llvm-3.9-runtime clang-format-3.9 python-clang-3.9 libfuzzer-3.9-dev
````

Then set v3.9 as default,
````
sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-3.9 60 --slave /usr/bin/clang++ clang++ /usr/bin/clang++-3.9
````

### libc++ library
Navigate to AirSim/cmake folder and run
````
bash getlibcxx.sh
````

## Building AirSim
Navigate to AirSim folder and type,

````
bash build.sh
````
This will create a `build_debug` folder containing the build output and the cmake generated make files.

## Clean (or reset) the build
To clean the build, just delete the `build_debug` folder.

## Running Unreal on Linux
Now you are ready to follow these instructions to get [Unreal working on Linux](https://wiki.unrealengine.com/Building_On_Linux#Clang) but note that everywhere
you see Clang3.5 in the Unreal documentation replace that with clang3.9.

## BashOnWindows
You can also compile from [BashOnWindows](https://msdn.microsoft.com/en-us/commandline/wsl/install_guide)
but make sure you are `not` using a `Visual Studio Command Prompt` because we don't want cmake to accidentally find VC++ and try and use that!
