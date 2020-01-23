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


        #  state = client.getMultirotorState()
         
         

        #  s = pprint.pformat(state)
        #  print("state: %s" % s)
          print("position")
          x = client.getMultirotorState();
     
          print("simSetCameraOrientation")
          position = client.getMultirotorState().kinematics_estimated.position;
          print(position)  
          print(float(position.x_val))  
          print(float(position.y_val))  
          print(float(position.z_val))  
          print(float(self.angle))  
          client.moveByVelocityZAsync(vx, vy, z_val,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, float(self.angle))).join()
          #client.moveToZAsync(z_val, 1).join()
          #airsim.DrivetrainType.MaxDegreeOfFreedom,airsim.YawMode(False, float(self.angle))
          #client.moveToPositionAsync(float(position.x_val),float(self.y_val), 1* float(self.z_val), 5).join() 
          #client.moveToPositionAsync(100*0.99,200*0.99, -330*0.99, 5,airsim.DrivetrainType.MaxDegreeOfFreedom,airsim.YawMode(False, float(fself.angle))).join() 
          #client.moveToPositionAsync(100*0.99,200*0.99, -330*0.99, 0.5, 10, airsim.DrivetrainType.MaxDegreeOfFreedom,airsim.YawMode(False, self.angle)).join() 
            #,airsim.YawMode(False, self.angle)).join()
          #client.rotateByYawRateAsync(30,2,"SimpleFlight").join()
          print("dam you")