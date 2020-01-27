import setup_path 
import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)




landed3 = client.getMultirotorState()
print(landed3)

landed = client.getMultirotorState().landed_state
if landed == airsim.LandedState.Landed:
    print("already landed...")
else:
    print("landing...")
    client.landAsync().join()

client.armDisarm(False)
client.enableApiControl(False)
