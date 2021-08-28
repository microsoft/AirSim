import airsim
import time

c = airsim.MultirotorClient()
c.confirmConnection()

c.simSetObjectMaterialFromTexture('OrangeBall', 'C:\\Users\\savempra\\Desktop\\jupi.jpg')