import sys
import setup_path 
import airsim
import pprint

class RotateToYaw:
      def __init__(self, angle):
          self.angle = angle
      def start(self):
          print("RotateToYaw")
          print(self.angle)
          print("armDisarm")
          client = airsim.MultirotorClient()
          client.confirmConnection()
          client.enableApiControl(True)
          client.armDisarm(True)
         
       #   client.rotateByYawRateAsync(float(self.angle)).join()
       #pitch, roll, throttle, yaw_rate
        #  client.moveByAngleThrottleAsync(float(self.angle),float(self.angle),float(self.angle),float(self.angle)).join()
          vx = 1
          vy = 0
          duration = 5

# Fly given velocity vector for 5 seconds
          speed = 1
          delay = duration * speed


          state = client.getMultirotorState()
         
         

          s = pprint.pformat(state)
          print("state: %s" % s)
          print("position")
          x = client.getMultirotorState();
     
          print("simSetCameraOrientation")
          z_val = client.getMultirotorState().kinematics_estimated.position.z_val;
          print(z_val)  
          client.moveByVelocityZAsync(vx, vy, z_val,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, float(self.angle))).join()
     
