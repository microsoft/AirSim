import airsim
import numpy as np
import time
from arkhe.hsi import HSI
from arkhe.fusion import FusionEngine
from arkhe.simulation import MorphogeneticSimulation
from arkhe.metasurface import MetasurfaceController
from arkhe.arkhe_types import CIEF

def main():
    print("🏛️ ARKHE(N) ENGINEERING SUITE - SENSORIUM DEMO")

    # Initialize Arkhe components
    hsi = HSI(size=0.5) # 0.5m voxels
    fusion = FusionEngine(hsi)
    sim = MorphogeneticSimulation(hsi)

    # Connect to AirSim
    client = airsim.MultirotorClient()
    try:
        client.confirmConnection()
        print("Connected to AirSim. Collecting multimodal data...")

        # 1. Collect LiDAR data
        lidar_data = client.getLidarData(lidar_name="LidarCustom")
        if len(lidar_data.point_cloud) >= 3:
            points = np.array(lidar_data.point_cloud).reshape(-1, 3)
            fusion.fuse_lidar(points)
            print(f"Fused {len(points)} LiDAR points into HSI.")

        # 2. Collect Depth and Thermal (Infrared) data
        responses = client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, pixels_as_float=True),
            airsim.ImageRequest("0", airsim.ImageType.Infrared, pixels_as_float=False, compress=False)
        ])

        if len(responses) >= 2:
            depth_map = airsim.list_to_2d_float_array(responses[0].image_data_float, responses[0].width, responses[0].height)
            thermal_image = np.frombuffer(responses[1].image_data_uint8, dtype=np.uint8).reshape(responses[1].height, responses[1].width)

            camera_info = client.simGetCameraInfo("0")
            fusion.fuse_multimodal(
                lidar_points=np.array(client.getLidarData(lidar_name="LidarCustom").point_cloud).reshape(-1, 3),
                thermal_image=thermal_image,
                depth_map=depth_map,
                camera_pose=camera_info.pose,
                camera_fov=camera_info.fov
            )
            print("Successfully fused Multimodal (LiDAR + Thermal + Depth) data.")

    except Exception as e:
        print(f"AirSim connection skipped or failed: {e}")
        print("Proceeding with Mock Data for demonstration...")

        # Generate mock LiDAR points for a 5x5m area
        mock_points = []
        for x in np.linspace(-2.5, 2.5, 20):
            for y in np.linspace(-2.5, 2.5, 20):
                z = 0.0 + 0.1 * np.random.randn()
                mock_points.append([x, y, z])
        fusion.fuse_lidar(np.array(mock_points))

        # Add some mock thermal "energy" (E) and information (I)
        # Center of the area has high energy
        center_voxel = hsi.add_point(0, 0, 0, genome_update={'e': 1.0, 'i': 1.0})
        center_voxel.rd_state = (0.5, 0.5) # Seed for simulation
        print("Mock data generated and seeded.")

    # Initialize Metasurface Controllers
    controllers = {coords: MetasurfaceController(v) for coords, v in hsi.voxels.items()}

    # Run Simulation Loop
    print("\nStarting Morphogenetic Field Simulation (Conscious States)...")
    for i in range(10):
        sim.step(dt=1.0)
        fusion.update_voxel_coherence()

        # Report status
        active_voxels = len(hsi.voxels)
        avg_phi = sum(v.phi for v in hsi.voxels.values()) / active_voxels if active_voxels > 0 else 0
        avg_phi_data = sum(v.phi_data for v in hsi.voxels.values()) / active_voxels if active_voxels > 0 else 0
        avg_phi_field = sum(v.phi_field for v in hsi.voxels.values()) / active_voxels if active_voxels > 0 else 0

        print(f"Step {i}: Voxels={active_voxels}, Φ_total={avg_phi:.4f}, Φ_data={avg_phi_data:.4f}, Φ_field={avg_phi_field:.4f}")

        # Metasurface consensus demonstration
        if i == 5:
            print("\n📡 Initiating Metasurface Consensus for high-coherence regions...")
            for coords, ctrl in controllers.items():
                if ctrl.voxel.phi > 0.4:
                    # Find neighbors controllers
                    nb_coords = hsi.get_neighbors(coords)
                    nb_ctrls = [controllers[c] for c in nb_coords if c in controllers]
                    if ctrl.propose_state(nb_ctrls, {"reflectivity": 1.0}):
                        print(f"  Consensus reached at {coords}: Metasurface state colapsed to reflective.")

        time.sleep(0.1)

    # Dynamic Simulation: Moving Vehicle (Opção A)
    print("\n🚗 Simulating Moving Vehicle across Vila Madalena malha...")
    vehicle_path = [(x, 0, 0) for x in np.linspace(-3, 3, 10)]
    for i, pos in enumerate(vehicle_path):
        x, y, z = pos
        # Simulate LiDAR + Thermal signature of a vehicle
        # High physicality (c), High energy (e)
        voxel = hsi.add_point(x, y, z, genome_update={'c': 0.8, 'e': 0.9, 'i': 0.5})
        voxel.phi_data = 0.95 # Highly coherent signature

        # Local Consensus for moving object
        nb_coords = hsi.get_neighbors(voxel.coords)
        nb_ctrls = [controllers[c] for c in nb_coords if c in controllers]
        if MetasurfaceController(voxel).propose_state(nb_ctrls, {"transparency": 0.2}):
             pass # Metasurface reacts to vehicle presence

        if i % 3 == 0:
            print(f"  Vehicle at {pos}: Voxel {voxel.coords} colapsed to 'Vehicle' signature.")
        time.sleep(0.05)

    print("\n✅ Arkhe(n) Sensorium Process Complete.")
    print("The terrain has been integrated into a conscious geometric organism.")

if __name__ == "__main__":
    main()
