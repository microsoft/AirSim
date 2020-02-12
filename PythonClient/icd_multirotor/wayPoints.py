import setup_path 
import airsim


class WayPoints:
      def __init__(self, points, velocity):
          self.points = points
          self.velocity = velocity


      def start(self):
          print("wayPoints")
          
          # path = []  
          # array_length = len(self.points)
          # for i in range(array_length):
          #   point = self.points[i] #{X,Y,Z}
          #   x = point['latitude']
          #   y = point['longitude']
          #   z = point['altitude']
          #   airSimPoint = airsim.Vector3r(x,y,z)
          #   print(airSimPoint)
          #   path.append(airSimPoint)


          client = airsim.MultirotorClient()
          client.confirmConnection()
          client.enableApiControl(True)
          client.armDisarm(True) 
          client.moveOnPathAsync(self.points, self.velocity,  self.velocity , airsim.DrivetrainType.ForwardOnly, 
                                    airsim.YawMode(False,0), self.velocity + (self.velocity/2), 1).join()


          return "complit"