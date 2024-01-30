import numpy as np

from typing import List, Tuple

from airsim.types import MultirotorClient
from airsim.types import Pose, Vector3r
from airsim.utils import to_eularian_angles, to_quaternion

class Gimbal:
    def __init__(self, cams : List[str]):
        """Gimbal

        Args:
            cams (List[str]): camera names in settings.json
        """        
        self.__cams = cams
        
    @property
    def cams(self):
        return self.__cams
    
    def discrete_rotation(self, camera : Pose, pitch : float, roll : float, yaw : float) -> Tuple:
        """A discretization for angle difference

        Args:
            camera (Pose): Current camera pose
            pitch (float): pitch in randians
            roll (float): roll in radians
            yaw (float): yaw in radians

        Returns:
            Tuple: discrezed positions for each DoF
        """        
        pitch_, roll_, yaw_ = to_eularian_angles(camera.orientation)
        
        d_pitch = np.arange(pitch_, pitch)
        d_roll = np.arange(roll_, roll)
        d_yaw = np.arange(yaw_, yaw)
        
        return d_pitch, d_roll, d_yaw
    
    def rotation(self, client : MultirotorClient, vehicle_name : str, pitch : float, roll : float, yaw : float):
        """Apply rotations for each camera

        Args:
            client (MultirotorClient): vehicle type connection
            vehicle_name (str): vehicle name in settings.json
            pitch (float): pitch in randians
            roll (float): roll in radians
            yaw (float): yaw in radians
        """        
        vehicle_pose = client.simGetVehiclePose(vehicle_name)
        xv, yv, zv = vehicle_pose.position
        
        camera_pose = client.simGetCameraInfo(self.__cams[0], vehicle_name).pose
        d_pitch, d_roll, d_yaw = self.discretize_rotation(camera_pose, pitch, roll, yaw)
        
        for p, r, y in zip(d_pitch, d_roll, d_yaw):
            pose = Pose(Vector3r(xv, yv, -zv), to_quaternion(p, r, y))
            for cam in self.__cams:
                client.simSetCameraPose(cam, pose, vehicle_name)