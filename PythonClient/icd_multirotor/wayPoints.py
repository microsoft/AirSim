import setup_path 
import airsim


class WayPoints:
      def __init__(self, points, velocity):
          self.points = points
          self.velocity = velocity


      def start(self):
          print("wayPoints")
          
          path = []  
          array_length = len(self.points)
          for i in range(array_length):
            point = self.points[i]
            airSimPoint = airsim.Vector3r(point[0], point[1], point[2])
            path.append(airSimPoint)


          client = airsim.MultirotorClient()
          client.confirmConnection()
          client.enableApiControl(True)
          client.armDisarm(True) 
          client.moveOnPathAsync(path, self.velocity, 154 , airsim.DrivetrainType.ForwardOnly, 
                                    airsim.YawMode(False,0), self.velocity + (self.velocity/2), 1).join()


          return "complit"