import setup_path 
import airsim
from airsim import Vector3r, Quaternionr, Pose
from airsim.utils import to_quaternion
import numpy as np
import time

client = airsim.VehicleClient()
client.confirmConnection()

# plot red arrows for 30 seconds
client.simPlotArrows(points_start = [Vector3r(x,y,0) for x, y in zip(np.linspace(0,10,20), np.linspace(0,0,20))], 
                        points_end = [Vector3r(x,y,0) for x, y in zip(np.linspace(0,10,20), np.linspace(10,10,20))], 
                        color_rgba = [1.0, 0.0, 1.0, 1.0], duration = 30.0, arrow_size = 10, thickness = 1)

# plot magenta arrows for 15 seconds
client.simPlotArrows(points_start = [Vector3r(x,y,-3) for x, y in zip(np.linspace(0,10,20), np.linspace(0,0,20))], 
                        points_end = [Vector3r(x,y,-5) for x, y in zip(np.linspace(0,10,20), np.linspace(10,20,20))], 
                        color_rgba = [1.0, 1.0, 0.0, 1.0], duration = 15.0, arrow_size = 20, thickness = 3)

# plot red arrows for 10 seconds
client.simPlotArrows(points_start = [Vector3r(x,y,z) for x, y, z in zip(np.linspace(0,10,20), np.linspace(0,0,20), np.linspace(-3,-10, 20))], 
                        points_end = [Vector3r(x,y,z) for x, y, z in zip(np.linspace(0,10,20), np.linspace(10,20,20), np.linspace(-5,-8, 20))], 
                        color_rgba = [1.0, 0.0, 0.0, 1.0], duration = 10.0, arrow_size = 100, thickness = 5)

# plot 2 white arrows which are persistent 
client.simPlotArrows(points_start = [Vector3r(x,y,-2) for x, y in zip(np.linspace(0,10,20), np.linspace(0,20,20))], 
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
client.simPlotStrings(strings = ["Microsoft AirSim" for i in range(5)], positions = [Vector3r(x,y,-1) for x, y in zip(np.linspace(0,5,5), np.linspace(0,0,5))], 
                        scale = 1, color_rgba=[1.0, 1.0, 1.0, 1.0], duration = 1200.0)

# client.simPlotTransforms(poses = [Pose(position_val=Vector3r(x,y,0), orientation_val=to_quaternion(pitch=0.0, roll=0.0, yaw=yaw)) for x, y, yaw in zip(np.linspace(0,10,10), np.linspace(0,0,10), np.linspace(0,np.pi,10))], 
#                         scale = 35, thickness = 5, duration = 1200.0, is_persistent = False)

# client.simPlotTransforms(poses = [Pose(position_val=Vector3r(x,y,0), orientation_val=to_quaternion(pitch=0.0, roll=roll, yaw=0.0)) for x, y, roll in zip(np.linspace(0,10,10), np.linspace(1,1,10), np.linspace(0,np.pi,10))], 
#                         scale = 35, thickness = 5, duration = 1200.0, is_persistent = False)

client.simPlotTransformsWithNames(poses = [Pose(position_val=Vector3r(x,y,0), orientation_val=to_quaternion(pitch=0.0, roll=0.0, yaw=yaw)) for x, y, yaw in zip(np.linspace(0,10,10), np.linspace(0,0,10), np.linspace(0,np.pi,10))], 
                                  names=["tf_yaw_" + str(idx) for idx in range(10)], tf_scale = 35, tf_thickness = 5, text_scale = 1, text_color_rgba = [1.0, 1.0, 1.0, 1.0], duration = 1200.0)

client.simPlotTransformsWithNames(poses = [Pose(position_val=Vector3r(x,y,0), orientation_val=to_quaternion(pitch=0.0, roll=roll, yaw=0.0)) for x, y, roll in zip(np.linspace(0,10,10), np.linspace(1,1,10), np.linspace(0,np.pi,10))], 
                                  names=["tf_roll_" + str(idx) for idx in range(10)], tf_scale = 35, tf_thickness = 5, text_scale = 1, text_color_rgba = [1.0, 1.0, 1.0, 1.0], duration = 1200.0)

client.simPlotTransformsWithNames(poses = [Pose(position_val=Vector3r(x,y,0), orientation_val=to_quaternion(pitch=pitch, roll=0.0, yaw=0.0)) for x, y, pitch in zip(np.linspace(0,10,10), np.linspace(-1,-1,10), np.linspace(0,np.pi,10))],
                                  names=["tf_pitch_" + str(idx) for idx in range(10)], tf_scale = 35, tf_thickness = 5, text_scale = 1, text_color_rgba = [1.0, 1.0, 1.0, 1.0], duration = 1200.0)

time.sleep(20.0)

client.simFlushPersistentMarkers()