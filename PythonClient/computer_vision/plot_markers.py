import setup_path 
import airsim
from airsim import Vector3r
import numpy as np

client = airsim.VehicleClient()
client.confirmConnection()

# plot 2 red arrows for 3 seconds
arrow_1_start, arrow_2_start = Vector3r(0,0,-1), Vector3r(0,0,-3)
arrow_1_end, arrow_2_end = Vector3r(1,1,-1), Vector3r(2,2,-3)
client.simPlotArrowList(points_start = [arrow_1_start, arrow_2_start], points_end = [arrow_1_end, arrow_2_end], 
                        color_rgba = [1.0, 0.0, 1.0, 1.0], duration = 3.0, arrow_size = 20, thickness = 4)

# plot 2 yellow arrows for 4 seconds
arrow_1_start, arrow_2_start = Vector3r(0,1,-1), Vector3r(0,1,-3)
arrow_1_end, arrow_2_end = Vector3r(4,5,-1), Vector3r(2,3,-3)
client.simPlotArrowList(points_start = [arrow_1_start, arrow_2_start], points_end = [arrow_1_end, arrow_2_end], 
                        color_rgba = [1.0, 1.0, 0.0, 1.0], duration = 4.0, arrow_size = 20, thickness = 3)

# plot 2 red arrows for 5 seconds
arrow_1_start, arrow_2_start = Vector3r(1,1,-2), Vector3r(1,1,-4)
arrow_1_end, arrow_2_end = Vector3r(-4,-4,-2), Vector3r(-2,-2,-4)
client.simPlotArrowList(points_start = [arrow_1_start, arrow_2_start], points_end = [arrow_1_end, arrow_2_end], 
                        color_rgba = [1.0, 0.0, 0.0, 1.0], duration = 5.0, arrow_size = 20, thickness = 2)

# plot 2 white arrows which are persistent 
arrow_1_start, arrow_2_start = Vector3r(2,2,-2), Vector3r(0,1,-4)
arrow_1_end, arrow_2_end = Vector3r(2,3,-2), Vector3r(2,4,-4)
client.simPlotArrowList(points_start = [arrow_1_start, arrow_2_start], points_end = [arrow_1_end, arrow_2_end], 
                        color_rgba = [1.0, 1.0, 1.0, 1.0], duration = 5.0, arrow_size = 20, thickness = 10, is_persistent = True)

# plot points 
client.simPlotPoints(points = [Vector3r(x,y,-2) for x, y in zip(np.linspace(0,10,20), np.linspace(0,20,20))],  color_rgba=[1.0, 0.0, 0.0, 1.0], size = 20, duration = 20.0, is_persistent = False)
client.simPlotPoints(points = [Vector3r(x,y,z) for x, y, z in zip(np.linspace(0,-10,20), np.linspace(0,-20,20), np.linspace(0,-5,20))],  color_rgba=[0.0, 0.0, 1.0, 1.0], size = 10, duration = 20.0, is_persistent = False)
client.simPlotPoints(points = [Vector3r(x,y,z) for x, y, z in zip(np.linspace(0,10,20), np.linspace(0,-20,20), np.linspace(0,-7,20))],  color_rgba=[1.0, 0.0, 1.0, 1.0], size = 15, duration = 20.0, is_persistent = False)
