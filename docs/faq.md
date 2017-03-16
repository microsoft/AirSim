
# FAQ

### Unreal editor is slow when it is not the active window

Go to Edit/Editor Preferences, select "All Settings" and type "CPU" in the sarch box. 
It should find the setting titled "Use Less CPU when in Background", and you want to uncheck this checkbox.

### Drone doesn't fly properly, it just goes crazy

This can be a machine performance issue, check your [hard drive performance](hard_drive.md).

### How to get cmake to use GCC 6 without making GCC 6 my default

Add these to your cmake command line `-D CMAKE_C_COMPILER=gcc-6 -D CMAKE_CXX_COMPILER=g++-6`.

### It is not finding my pixhawk hardware

Check your settings.json file for this line "SerialPort":"*,115200".  The asterix here means "find any 
serial port that looks like a Pixhawk device, but this doesn't always work for all types of pixhawk hardware.
So on Windows you can find the actual COM port using Device Manager, look under "Ports (COM & LPT), plug the 
device in and see what new COM port shows up.  Let's say you see a new port named "USB Serial Port (COM5)". 
Well, then change the SerialPort setting to this: "SerialPort":"COM5,115200".  

On Linux, the device can be found by running "ls /dev/serial/by-id" if you see a device name listed that looks
like this `usb-3D_Robotics_PX4_FMU_v2.x_0-if00` then you can use that name to connect, like this:
"SerialPort":"/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00".  Note this long name is actually a symbolic link to the real 
name, if you use "ls -l ..." you can find that symbolic link, it is usually something like "/dev/ttyACM0",
so this will also work "SerialPort":"/dev/ttyACM0,115200".  But that mapping is similar to windows, it is
automatically assigned and can change, whereas the long name will work even if the actual tty serial device
mapping changes.
