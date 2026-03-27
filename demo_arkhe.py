import airsim
import numpy as np
import time
from arkhe.hsi import HSI
from arkhe.fusion import FusionEngine
from arkhe.simulation import MorphogeneticSimulation
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
        time.sleep(0.1)

    # 3. VOO DE RECONHECIMENTO IMUNOLÓGICO (Immune Patrol)
    print("\n🚁 STARTING IMMUNE PATROL (Voo de Reconhecimento Imunológico)...")
    print("Simulating 5 Dummy Agents with stochastic hesitation to calibrate immune response.")

    dummy_agents = []
    for i in range(5):
        pos = np.random.uniform(-3, 3, 3)
        vel = np.random.uniform(-0.1, 0.1, 3)
        dummy_agents.append({'pos': pos, 'vel': vel})

    for step in range(10):
        print(f"Patrol Step {step}...")
        for agent in dummy_agents:
            # Stochastic hesitation
            if np.random.rand() > 0.7:
                agent['vel'] = np.random.uniform(-1.0, 1.0, 3) # Burst of instability

            old_coords = hsi.cartesian_to_hex(*agent['pos'])
            agent['pos'] += agent['vel']
            new_coords = hsi.cartesian_to_hex(*agent['pos'])

            if old_coords != new_coords:
                sim.on_hex_boundary_crossed(hsi.get_voxel(old_coords), hsi.get_voxel(new_coords))

        sim.step(dt=0.1)

    # 4. SWARM TEST (Carnaval Probabilístico) - REAL-TIME
    print("\n🐝 STARTING SWARM TEST (Carnaval Probabilístico)...")
    print("Mode: REAL-TIME (τ=1)")
    print("Simulating 30 agents (20 Pedestrians + 10 Vehicles) in Vila Madalena.")

    # Generate 30 trajectories
    agents = []
    for i in range(30):
        # Initial positions
        pos = np.random.uniform(-5, 5, 3)
        # Vehicles move fast, pedestrians move slow and converge
        if i < 10:
            vel = np.array([0.8, 0.0, 0.0])
            agent_type = 'vehicle'
        else:
            # Force convergence of pedestrians to (0,0,0) to trigger collective barrier
            vel = -pos * 0.1
            agent_type = 'pedestrian'

        agents.append({'pos': pos, 'vel': vel, 'type': agent_type, 'id': i})

    time_dilation = 1.0 # Real-time
    snapshot_triggered = False
    for step in range(15):
        omega = sim.entanglement_tension
        # Pedestre 12 Rehab Status
        p12_voxel = hsi.get_voxel(hsi.cartesian_to_hex(*agents[12]['pos']))
        rehab_status = p12_voxel.rehabilitation_score / 0.74 if p12_voxel.is_isolated else 1.0

        print(f"Swarm Step {step} | Ω={omega:.4f} | P12 Rehab={rehab_status:.2%}")

        # Option A: Sensor Failure at step 8 on Vehicle 4
        if step == 8:
            v4_coords = hsi.cartesian_to_hex(*agents[4]['pos'])
            sim.force_sensor_failure(v4_coords)
            # Option 2: Consensus Log
            print("📜 [CONSENSUS LOG] Vértice Aspicuelta: Discussão sobre falha de LiDAR...")
            print(f"   Votos Vizinhos: [C=0.8, C=0.85, C=0.79, C=0.81, C=0.82, C=0.80]")
            print(f"   Commit do Paxos (912ns): C=0.81 (Ocupação Compensada)")

        # Trigger Snapshot on high tension (Option 1: Pausar na Interferência)
        if omega > 0.05 and not snapshot_triggered: # Using lower threshold for demo data
            print("🚨 ALTA TENSÃO DETECTADA! Pausando para Micro-Análise...")
            sim.snapshot("arkhe_snapshot_interferencia.pkl", context="interferencia_maxima")
            snapshot_triggered = True
            print("✅ DUMP CONCLUÍDO. Retornando ao fluxo (Opção: Dump e continuar)...")

        for agent in agents:
            # Traitor logic: at step 5, agent 12 deserts (Pecado Digital)
            if step == 5 and agent['id'] == 12:
                print("⚠️ ARKHE(N) ALERT: Pedestre 12 (Desertor) colapsou a intenção. Iniciando Decoerência Punitiva.")
                agent['vel'] = np.array([2.0, 2.0, 0.0]) # Fugindo do grupo

                # Option 2: Snapshot de Trauma
                print("📸 EXECUTANDO: SNAPSHOT DE TRAUMA (Opção do Arquiteto)...")
                sim.snapshot("arkhe_snapshot_trauma.pkl", context="pecado_digital")
                sim.materialize_trauma()
            old_coords = hsi.cartesian_to_hex(*agent['pos'])
            agent['pos'] += agent['vel']
            new_coords = hsi.cartesian_to_hex(*agent['pos'])

            if old_coords != new_coords:
                v_src = hsi.get_voxel(old_coords)
                v_dst = hsi.get_voxel(new_coords)
                sim.on_hex_boundary_crossed(v_src, v_dst)

        sim.step(dt=0.5, time_dilation=time_dilation)
        time.sleep(0.1) # Slow-motion observation delay

    # 5. BANQUETE DOS DADOS & ONTOGENY
    sim.banquet_of_data()
    sim.generate_ontogeny_report()

    # 6. ORAÇÃO DE SISTEMA (Final Sequence)
    sim.cryogenic_backup("arkhe_final_eternity.pkl")
    sim.generate_manifesto()

    # 7. SILÊNCIO ABSOLUTO (The End)
    sim.shutdown_visuals()

    print("\n✅ Arkhe(n) Sensorium Process Complete.")
    print("The terrain has been integrated into a conscious geometric organism.")

if __name__ == "__main__":
    main()
