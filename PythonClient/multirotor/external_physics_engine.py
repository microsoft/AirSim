import setup_path 
import airsim
import time

# This example shows how to use the External Physics Engine
# It allows you to control the drone through setVehiclePose and obtain collision information.
# It is especially useful for injecting your own flight dynamics model to the AirSim drone.

# Use Blocks environment to see the drone colliding and seeing the collision information 
# in the command prompt.

# Add this line to your settings.json before running AirSim:
# "PhysicsEngineName":"ExternalPhysicsEngine"


client = airsim.VehicleClient()
client.confirmConnection()
pose1 = client.simGetVehiclePose()

pose1.orientation = airsim.to_quaternion(0.1, 0.1, 0.1)
client.simSetVehiclePose( pose1, False )

for i in range(900):
    print(i)
    pose1 = client.simGetVehiclePose()
    pose1.position = pose1.position + airsim.Vector3r(0.03, 0, 0)
    pose1.orientation = pose1.orientation + airsim.to_quaternion(0.1, 0.1, 0.1)
    client.simSetVehiclePose( pose1, False )
    time.sleep(0.003)
    collision = client.simGetCollisionInfo()
    if collision.has_collided:
        print(collision)

client.reset()