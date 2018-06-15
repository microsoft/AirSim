from AirSimClient import *

client = MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

client.armDisarm(True)

landed = client.getLandedState()
if landed == LandedState.Landed:
    print("taking off...")
    client.takeoff()
else:
    print("already flying...")
    client.hover()
