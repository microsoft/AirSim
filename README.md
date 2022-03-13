This is a fork of AirSim providing simulation for Clearpath [Warthog](https://clearpathrobotics.com/warthog-unmanned-ground-vehicle-robot/)

Clone this repository and build AirSim following the instructions [AirSim](https://microsoft.github.io/AirSim/build_windows/)  

If you are using Linux follow the instructions to build AirSim on Linux.

To use AirSim (warthog Included) in your unreal project follow the instructions [here](https://microsoft.github.io/AirSim/unreal_custenv/)

If you want to use default blocks environment provided with AirSim follow [these](https://microsoft.github.io/AirSim/unreal_blocks/) instructions.

Once you build the project (by following any of the above two instructions) and press play you will see the message "Would you like to use car simulation? Choose no to use quadrotor simulation." Choose "NO" and you will see Warthog Instead of quadrotor(Work in progress !!).

Look at the file [warthog.py](https://github.com/akhil22/AirSim/blob/warthog/PythonClient/warthog/warthog.py) to understand how to control warthog. You can use Joystick to command linear and angular velocities(left analog stick up and down for linear velocity and right analog stick sideways for angular velocities) after running warthog.py to command linear and angular velocities after running warthog.py. This file is under progress and will be updated with detailed instructions soon.  

Thank You !!
