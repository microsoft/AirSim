import setup_path 
import airsim
from airsim import Vector3r, Quaternionr, Pose
from airsim.utils import to_quaternion
import numpy as np
import time

client = airsim.VehicleClient()
client.confirmConnection()

# plot red arrows for 30 seconds
client.simPlotArrowList(points_start = [Vector3r(x,y,0) for x, y in zip(np.linspace(0,10,20), np.linspace(0,0,20))], 
                        points_end = [Vector3r(x,y,0) for x, y in zip(np.linspace(0,10,20), np.linspace(10,10,20))], 
                        color_rgba = [1.0, 0.0, 1.0, 1.0], duration = 30.0, arrow_size = 10, thickness = 1)

# plot magenta arrows for 15 seconds
client.simPlotArrowList(points_start = [Vector3r(x,y,-3) for x, y in zip(np.linspace(0,10,20), np.linspace(0,0,20))], 
                        points_end = [Vector3r(x,y,-5) for x, y in zip(np.linspace(0,10,20), np.linspace(10,20,20))], 
                        color_rgba = [1.0, 1.0, 0.0, 1.0], duration = 15.0, arrow_size = 20, thickness = 3)

# plot red arrows for 10 seconds
client.simPlotArrowList(points_start = [Vector3r(x,y,z) for x, y, z in zip(np.linspace(0,10,20), np.linspace(0,0,20), np.linspace(-3,-10, 20))], 
                        points_end = [Vector3r(x,y,z) for x, y, z in zip(np.linspace(0,10,20), np.linspace(10,20,20), np.linspace(-5,-8, 20))], 
                        color_rgba = [1.0, 0.0, 0.0, 1.0], duration = 10.0, arrow_size = 100, thickness = 5)

# plot 2 white arrows which are persistent 
client.simPlotArrowList(points_start = [Vector3r(x,y,-2) for x, y in zip(np.linspace(0,10,20), np.linspace(0,20,20))], 
                        points_end = [Vector3r(x,y,-5) for x, y in zip(np.linspace(3,17,20), np.linspace(5,28,20))], 
                        color_rgba = [1.0, 1.0, 1.0, 1.0], duration = 5.0, arrow_size = 100, thickness = 1, is_persistent = True)

# plot points 
client.simPlotPoints(points = [Vector3r(x,y,-5) for x, y in zip(np.linspace(0,-10,20), np.linspace(0,-20,20))], color_rgba=[1.0, 0.0, 0.0, 1.0], size = 25, duration = 20.0, is_persistent = False)
client.simPlotPoints(points = [Vector3r(x,y,z) for x, y, z in zip(np.linspace(0,-10,20), np.linspace(0,-20,20), np.linspace(0,-5,20))], color_rgba=[0.0, 0.0, 1.0, 1.0], size = 10, duration = 20.0, is_persistent = False)
client.simPlotPoints(points = [Vector3r(x,y,z) for x, y, z in zip(np.linspace(0,10,20), np.linspace(0,-20,20), np.linspace(0,-7,20))], color_rgba=[1.0, 0.0, 1.0, 1.0], size = 15, duration = 20.0, is_persistent = False)

# plot line strip. 0-1, 1-2, 2-3
client.simPlotLineStrip(points = [Vector3r(x,y,-5) for x, y in zip(np.linspace(0,-10,10), np.linspace(0,-20,10))], color_rgba=[1.0, 0.0, 0.0, 1.0], thickness = 5, duration = 30.0, is_persistent = False)

# plot line list. 0-1, 2-3, 4-5. Must be even. 
client.simPlotLineList(points = [Vector3r(x,y,-7) for x, y in zip(np.linspace(0,-10,10), np.linspace(0,-20,10))], color_rgba=[1.0, 0.0, 0.0, 1.0], thickness = 5, duration = 30.0, is_persistent = False)

# plot transforms 
client.simPlotTransform(poses = [Pose(position_val=Vector3r(x,y,-7), orientation_val=to_quaternion(pitch=0.0, roll=0.0, yaw=np.pi/2)) for x, y in zip(np.linspace(0,-10,10), np.linspace(0,-20,10))], 
                        scale = 35, thickness = 5, duration = 10.0, is_persistent = False)

client.simPlotTransform(poses = [Pose(position_val=Vector3r(x,y,-5), orientation_val=to_quaternion(pitch=0.0, roll=0.0, yaw=0.0)) for x, y in zip(np.linspace(0,-10,10), np.linspace(0,-20,10))], 
                        scale = 35, thickness = 5, duration = 10.0, is_persistent = False)

