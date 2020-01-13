import setup_path 
import airsim


class MoveToPosition:
      def __init__(self, initX, initY, initZ):
          self.initX = initX
          self.initY = initY
          self.initZ = initZ

      def start(self):
          print("MoveToPositionStart")
          client = airsim.MultirotorClient()
          client.confirmConnection()
          client.enableApiControl(True)
          client.armDisarm(True) 
          print( -1 * float(self.initZ))
          client.moveToPositionAsync(float(self.initX),float(self.initY), -1 * float(self.initZ), 5).join()
          return "complit"