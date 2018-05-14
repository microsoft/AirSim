from AirSimClient import *

client = MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

pos = client.getPosition()
print("x={}, y={}, z={}".format(pos.x_val, pos.y_val, pos.z_val))

pos = client.getPitchRollYaw()
print("pitch={}, roll={}, yaw={}".format(pos[0], pos[1], pos[2]))