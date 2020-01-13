import sys
import setup_path 
import airsim
import pprint
 

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

camera_info = client.simGetCameraInfo("0")
cam = pprint.pformat(camera_info)
print("state: %s" % cam)
print("gimbal start")  
print(camera_info)
client.simSetCameraOrientation("0", airsim.to_quaternion(0.261799 * 10, 0, 0)); #radians

        #  camera_info = client.simGetCameraInfo(str(0))
        #  print("CameraInfo %d: %s" % (0, pp.pprint(camera_info)))

print("gimbal end")  