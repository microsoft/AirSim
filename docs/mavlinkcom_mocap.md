# Welcome to MavLinkMoCap

This folder contains the MavLinkMoCap library which connects to a OptiTrack camera system
for accurate indoor location.

## Dependencies:
* [OptiTrack Motive](http://www.optitrack.com/products/motive/).
* [MavLinkCom](mavlinkcom.md).

### Setup RigidBody

First you need to define a RigidBody named 'Quadrocopter' using Motive.
See [Rigid_Body_Tracking](http://wiki.optitrack.com/index.php?title=Rigid_Body_Tracking).

### MavLinkTest

Use MavLinkTest to talk to your PX4 drone, with "-server:addr:port", for example, when connected
to drone wifi use: 

    MavLinkMoCap -server:10.42.0.228:14590 "-project:D:\OptiTrack\Motive Project 2016-12-19 04.09.42 PM.ttp" 

This publishes the ATT_POS_MOCAP messages and you can proxy those through to the PX4 by running 
MavLinkTest on the dronebrain using:

    MavLinkTest -serial:/dev/ttyACM0,115200 -proxy:10.42.0.228:14590

Now the drone will get the ATT_POS_MOCAP and you should see the light turn green meaning it is
now has a home position and is ready to fly.