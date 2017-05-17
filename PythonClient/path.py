from PythonClient import *
import sys
import time

client = AirSimClient('127.0.0.1')

# see https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo

client.moveOnPath([(0,-253,-5),(125,-253,-5),(125,0,-5),(0,0,-5)], 12, DrivetrainType.ForwardOnly, YawMode(False,0), 15, 1)
