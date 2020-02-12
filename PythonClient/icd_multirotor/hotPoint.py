import setup_path 
import airsim


class HotPoint:
      def __init__(self, initX, initY, initZ):
          self.initX = initX
          self.initY = initY
          self.initZ = initZ

      def start(self):
          print("hot Point")
          client = airsim.MultirotorClient()
          client.confirmConnection()
          client.enableApiControl(True)
          client.armDisarm(True) 
          client.moveToPositionAsync(float(self.initX),float(self.initY), -1 * float(self.initZ), 5).join()
          #client.moveToPositionAsync(float(self.initX),float(self.initY), float(self.initZ), 10).join()

          print("simSetCameraOrientation")
          position = client.getMultirotorState().kinematics_estimated.position
          print(position)  

          return "complit"