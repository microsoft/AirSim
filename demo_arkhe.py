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

    # Dynamic Simulation: "The First Cry" (Opção A)
    print("\n🚗💨 Simulating 'The First Cry': Moving Vehicle & Pedestrian (Aspicuelta x Harmonia)...")

    # 1. Moving Vehicle: West to East
    vehicle_path = [(x, 0, 0) for x in np.linspace(-4, 4, 8)]
    # 2. Moving Pedestrian: South to North - Meeting at (0,0)
    pedestrian_path = [(0, y, 0) for y in np.linspace(-4, 4, 8)]

    telemetry_report = []

    for i in range(8):
        # Update simulation state
        sim.step(dt=1.0)
        fusion.update_voxel_coherence()

        # Vehicle move
        vx, vy, vz = vehicle_path[i]
        v_voxel = hsi.add_point(vx, vy, vz, genome_update={'c': 0.9, 'e': 0.4, 'i': 0.8})
        v_voxel.phi_data = 0.98

        # Pedestrian move
        px, py, pz = pedestrian_path[i]
        p_voxel = hsi.add_point(px, py, pz, genome_update={'c': 0.2, 'e': 0.9, 'i': 0.9})
        p_voxel.phi_data = 0.96

        # Local Consensus for both
        for vox in [v_voxel, p_voxel]:
            if vox.coords not in controllers:
                controllers[vox.coords] = MetasurfaceController(vox)
            ctrl = controllers[vox.coords]
            nb_coords = hsi.get_neighbors(vox.coords)
            nb_ctrls = [controllers[c] for c in nb_coords if c in controllers]

            # Vehicle triggers low transparency, Pedestrian triggers high reflectivity (cooling)
            target = {"transparency": 0.2} if vox == v_voxel else {"reflectivity": 1.0}
            if ctrl.propose_state(nb_ctrls, target):
                pass

        telemetry_report.append({
            "step": i,
            "vehicle_pos": vehicle_path[i],
            "vehicle_phi": v_voxel.phi,
            "vehicle_amplitudes": v_voxel.state,
            "pedestrian_pos": pedestrian_path[i],
            "pedestrian_phi": p_voxel.phi,
            "pedestrian_amplitudes": p_voxel.state
        })

        if i % 2 == 0:
            print(f"  Step {i}: Vehicle @ {vehicle_path[i]} (C-Heavy), Pedestrian @ {pedestrian_path[i]} (E-Heavy)")
        time.sleep(0.05)

    print("\n📊 TELEMETRY REPORT: ARKHE(N) SENSORIUM FIRST CRY")
    print("-" * 100)
    for entry in telemetry_report:
        v_amp = np.abs(entry['vehicle_amplitudes'])
        p_amp = np.abs(entry['pedestrian_amplitudes'])
        v_coords = hsi.cartesian_to_hex(*entry['vehicle_pos'])
        p_coords = hsi.cartesian_to_hex(*entry['pedestrian_pos'])

        conflict_v = hsi.voxels[v_coords].conflict_level
        conflict_p = hsi.voxels[p_coords].conflict_level

        print(f"Step {entry['step']} | V @ {v_coords} Phi: {entry['vehicle_phi']:.4f} Conflict: {conflict_v:.2f}")
        print(f"       | P @ {p_coords} Phi: {entry['pedestrian_phi']:.4f} Conflict: {conflict_p:.2f}")

        # Check consensus latency for vehicle
        v_ctrl = controllers.get(v_coords)
        if v_ctrl:
            print(f"       | Consensus Latency: {v_ctrl.consensus_latency_ms:.4f}ms | Radiative Cooling: {v_ctrl.current_property['radiative_cooling']}")
    print("-" * 100)

    print("\n✅ Arkhe(n) Sensorium Process Complete.")
    print("The terrain has been integrated into a conscious geometric organism.")

if __name__ == "__main__":
    main()
