from AirSimClient import *

# connect to the AirSim simulator
client = MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

print("Taking off")
client.moveByVelocityZ(0, 0, -20, 8)
time.sleep(3)    

for i in range(1, 6):
    print("Starting command to run for 15sec")
    client.moveByVelocityZ(-1*i, -1*i, -20-i, 15)
    time.sleep(5) #run
    print("Pausing after 5sec")
    client.simPause(True)
    time.sleep(5) #paused
    print("Restarting command to run for 7.5sec")
    client.simContinueForTime(7.5) 
    time.sleep(10)
    print("Finishing rest of the command")
    client.simPause(False)
    time.sleep(15)
    print("Finished cycle")


    
    
