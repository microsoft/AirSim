
# FAQ

### Unreal editor is slow when it is not the active window

Go to Edit/Editor Preferences, select "All Settings" and type "CPU" in the sarch box. 
It should find the setting titled "Use Less CPU when in Background", and you want to uncheck this checkbox.

### Drone doesn't fly properly, it just goes crazy

This can be a machine performance issue, check your [hard drive performance](hard_drive.md).

### How to get cmake to use GCC 6 without making GCC 6 my default

Add these to your cmake command line `-D CMAKE_C_COMPILER=gcc-6 -D CMAKE_CXX_COMPILER=g++-6`.

### LINK : fatal error LNK1104: cannot open file 'libboost_filesystem-vc140-mt-gd-1_63.lib'

This error happens on windows if you forgot to build the debug version of the boost library.
See [install boost](install_boost.md) for the full boost build (b2) command line.

