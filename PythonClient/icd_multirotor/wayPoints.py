import setup_path 
import airsim


class WayPoints:
      def __init__(self, points, velocity):
          self.points = points
          self.velocity = velocity


      def start(self):
          print("wayPoints")
          

          client = airsim.MultirotorClient()
          client.confirmConnection()
          client.enableApiControl(True)
          client.armDisarm(True) 

          # client.moveOnPathAsync(self.points, self.velocity,  self.velocity , airsim.DrivetrainType.ForwardOnly, 
          #                             airsim.YawMode(False,0), self.velocity + (self.velocity/2), 1).join()
          client.moveOnPathAsync(self.points,
                          12).join()

          return "complit"

          