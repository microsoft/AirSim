import numpy as np
import time
from dataclasses import dataclass
from arkhe import HSI, FusionEngine, HumanPerspectiveEngine, MorphogeneticSimulation, MetasurfaceController

@dataclass
class Vec3:
    x_val: float
    y_val: float
    z_val: float

@dataclass
class Pose:
    position: Vec3

def run_sensorium_demo():
    print("🏛️ ARKHE(N) SENSORIUM: INTEGRATED TERRAIN PERSPECTIVE")
    print("-" * 60)

    # 1. Initialize HSI and Engines
    hsi = HSI(size=1.0)
    fusion = FusionEngine(hsi)
    vision = HumanPerspectiveEngine(hsi)
    sim = MorphogeneticSimulation(hsi)

    # 2. Mock Data Generation
    print("📡 Generating Multimodal Data (LIDAR, Thermal, Depth)...")

    # LIDAR: A cluster of points representing a vehicle at (10, 10, 2)
    lidar_points = np.random.normal(loc=[10, 10, 2], scale=0.5, size=(100, 3))

    # Thermal & Depth: 100x100 maps
    thermal_map = np.zeros((100, 100), dtype=np.uint8)
    # Heat signature where the vehicle is
    thermal_map[40:60, 40:60] = 200

    depth_map = np.ones((100, 100), dtype=np.float32) * 10.0
    depth_map[40:60, 40:60] = 5.0 # Closer object (vehicle)

    camera_pose = Pose(position=Vec3(0, 0, 5))

    # 3. Data Fusion
    print("🧬 Fusing Data into HSI...")
    fusion.fuse_multimodal(lidar_points, thermal_map, depth_map, camera_pose, camera_fov=90)
    fusion.update_voxel_coherence()

    # 4. Human Perspective / Object Recognition
    print("👁️ Identifying Objects and Context...")
    vision.identify_objects(similarity_threshold=0.8)
    print(vision.get_contextual_summary())

    # 5. Field Simulation and Metasurface Response
    print("🌊 Simulating Conscious Fields and Metasurface Adaptations...")
    # Target some voxels of the identified vehicle for metasurface adjustment
    vehicle_voxels = vision.objects.get(0, [])

    if vehicle_voxels:
        target_coords = vehicle_voxels[0]
        voxel = hsi.get_voxel(target_coords)
        controller = MetasurfaceController(voxel)

        # Propose state change: High reflectivity to signal presence
        print(f"  [Metasurface] Adjusting state for {voxel.object_label} at {target_coords}")
        controller.current_property["reflectivity"] = 0.9

        # Step the morphogenetic simulation
        sim.step(dt=0.1)

    print("-" * 60)
    print("✅ SENSORIUM CYCLE COMPLETE")
    print(f"Global Coherence Φ: {np.mean([v.phi for v in hsi.voxels.values()]):.4f}")

if __name__ == "__main__":
    run_sensorium_demo()
