from PythonClient import *

# connect to the AirSim simulator 
client = AirSimClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)


AirSimClient.wait_key('Press any key to takeoff')
client.takeoff()

AirSimClient.wait_key('Press any key to take images')
responses = client.simGetImages([ImageRequest(0, AirSimImageType.DepthVis), ImageRequest(1, AirSimImageType.DepthMeters, True)])

print('Retrieved images: %d \n', len(responses))

print("done")
#for response in responses:
#    print(response.camera_position)
#    print(response.camera_position)

    