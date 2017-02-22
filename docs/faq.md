
# FAQ

### <font color='#00007c'>Unreal editor is slow when it is not the active window</font>

Go to Edit/Editor Preferences, select "All Settings" and type "CPU" in the sarch box. 
It should find the setting titled "Use Less CPU when in Background", and you want to uncheck this checkbox.

### <font color='#00007c'>Drone doesn't fly properly, it just goes crazy</font>

This can be a machine performance issue, check your [hard drive performance](hard_drive.md).

### <font color='#00007c'>How to get cmake to use GCC 6 without making GCC 6 my default</font>

Add these to your cmake command line `-D CMAKE_C_COMPILER=gcc-6 -D CMAKE_CXX_COMPILER=g++-6`.

