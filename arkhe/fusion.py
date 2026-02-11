import numpy as np
from typing import List, Tuple
from .arkhe_types import HexVoxel, CIEF
from .hsi import HSI

class FusionEngine:
    """
    FusionEngine: Unifies LIDAR, Thermal (IR), and Depth data into the HSI.
    """
    def __init__(self, hsi: HSI):
        self.hsi = hsi

    def fuse_lidar(self, points: np.ndarray):
        """
        Processes LIDAR point cloud and updates the C (Construction) component.
        Implements 'Prótese Cognitiva' if sensor health is low.
        """
        for i in range(len(points)):
            x, y, z = points[i]
            coords = self.hsi.cartesian_to_hex(x, y, z)
            voxel = self.hsi.get_voxel(coords)

            if voxel.sensor_health > 0.5:
                # Normal operation: Each LIDAR point reinforces physicality (C)
                voxel.genome.c += 0.1
            else:
                # Prótese Cognitiva: Borrow structure from neighbors
                neighbors = self.hsi.get_neighbors(coords)
                nb_c = [self.hsi.voxels[nb].genome.c for nb in neighbors if nb in self.hsi.voxels]
                if nb_c:
                    voxel.genome.c = np.mean(nb_c)
                    voxel.phi_data = 0.9 # Artificially high to represent 'faith' in consensus

    def fuse_thermal(self, thermal_image: np.ndarray, depth_map: np.ndarray, camera_pose, camera_fov_deg: float):
        """
        Processes Thermal (IR) image and updates the E (Energy) component using Depth for 3D projection.
        """
        h, w = thermal_image.shape
        fov_rad = np.deg2rad(camera_fov_deg)
        f = w / (2 * np.tan(fov_rad / 2))

        # Sample some pixels for performance in this demo
        step = 10
        for i in range(0, h, step):
            for j in range(0, w, step):
                d = depth_map[i, j]
                if d > 100 or d < 0.1: continue

                # Project pixel to camera coordinates
                z_c = d
                x_c = (j - w/2) * z_c / f
                y_c = (i - h/2) * z_c / f

                # Convert to world coordinates (simplified: assuming camera at pose)
                # In real scenario, multiply by camera rotation matrix and add translation
                x_w = camera_pose.position.x_val + x_c
                y_w = camera_pose.position.y_val + y_c
                z_w = camera_pose.position.z_val + z_c

                intensity = thermal_image[i, j] / 255.0
                coords = self.hsi.cartesian_to_hex(x_w, y_w, z_w)
                voxel = self.hsi.get_voxel(coords)
                if voxel.sensor_health > 0.5:
                    voxel.genome.e += intensity
                else:
                    # Compensation
                    neighbors = self.hsi.get_neighbors(coords)
                    nb_e = [self.hsi.voxels[nb].genome.e for nb in neighbors if nb in self.hsi.voxels]
                    if nb_e: voxel.genome.e = np.mean(nb_e)

    def fuse_depth(self, depth_map: np.ndarray, camera_pose, camera_fov_deg: float):
        """
        Processes Depth map and updates the I (Information) component.
        """
        h, w = depth_map.shape
        fov_rad = np.deg2rad(camera_fov_deg)
        f = w / (2 * np.tan(fov_rad / 2))

        step = 10
        for i in range(0, h, step):
            for j in range(0, w, step):
                d = depth_map[i, j]
                if d > 100 or d < 0.1: continue

                x_c = (j - w/2) * d / f
                y_c = (i - h/2) * d / f

                x_w = camera_pose.position.x_val + x_c
                y_w = camera_pose.position.y_val + y_c
                z_w = camera_pose.position.z_val + d

                # Each depth point reinforces Information (I)
                coords = self.hsi.cartesian_to_hex(x_w, y_w, z_w)
                voxel = self.hsi.get_voxel(coords)
                if voxel.sensor_health > 0.5:
                    voxel.genome.i += 0.1
                else:
                    # Compensation
                    neighbors = self.hsi.get_neighbors(coords)
                    nb_i = [self.hsi.voxels[nb].genome.i for nb in neighbors if nb in self.hsi.voxels]
                    if nb_i: voxel.genome.i = np.mean(nb_i)

    def fuse_multimodal(self, lidar_points: np.ndarray, thermal_image: np.ndarray, depth_map: np.ndarray, camera_pose, camera_fov: float):
        """
        Unified fusion kernel.
        """
        self.fuse_lidar(lidar_points)
        self.fuse_depth(depth_map, camera_pose, camera_fov)
        self.fuse_thermal(thermal_image, depth_map, camera_pose, camera_fov)

    def update_voxel_coherence(self):
        """
        Calculates Phi_data (Coherence) for each voxel based on the integration of data.
        Phi = 1 - S/log(6) where S is entropy.
        """
        for voxel in self.hsi.voxels.values():
            g = voxel.genome
            vals = np.array([g.c, g.i, g.e, g.f])
            if np.sum(vals) > 0:
                probs = vals / np.sum(vals)
                entropy = -np.sum(probs * np.log(probs + 1e-9))
                voxel.phi_data = 1.0 - (entropy / np.log(6))
            else:
                voxel.phi_data = 0.0
