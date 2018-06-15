from AirSimClient import *

client = MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

landed = client.getLandedState()
if landed == LandedState.Landed:
    print("already landed...")
else:
    print("landing...")
    client.land()

client.armDisarm(False)
client.enableApiControl(False)
