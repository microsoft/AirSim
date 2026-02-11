import airsim
import numpy as np
import time
from arkhe.hsi import HSI
from arkhe.fusion import FusionEngine
from arkhe.simulation import MorphogeneticSimulation
from arkhe.metasurface import MetasurfaceController
from arkhe.memory import HSISnapshot, MemoryAnalyzer
from arkhe.arkhe_types import CIEF

def main():
    print("🏛️ ARKHE(N) ENGINEERING SUITE - SENSORIUM SWARM DEMO")

    # Initialize Arkhe components
    hsi = HSI(size=0.5)
    fusion = FusionEngine(hsi)
    sim = MorphogeneticSimulation(hsi, learning_rate=0.05)

    # 0. Baseline Snapshot
    baseline = HSISnapshot(hsi)

    # Initialize Metasurface Controllers
    controllers = {}

    # 1. Setup Swarm: 20 Pedestrians, 10 Vehicles
    print("\n🎭🎭 Simulating 'Carnaval Quântico' (Option A: Interferência Coletiva)")
    print("Modo: Slow-Motion Quântico (Time Dilation x100)")
    print("Location: Crossing Aspicuelta x Harmonia")

    num_peds = 20
    num_vehs = 10
    agents = []

    # Pedestrians coming from South, moving North
    for _ in range(num_peds):
        start = np.array([np.random.uniform(-2, 2), -5, 0])
        end = np.array([start[0], 5, 0])
        agents.append({"type": "pedestrian", "path": [start + (end-start)*t for t in np.linspace(0, 1, 15)]})

    # Vehicles coming from West, moving East
    for _ in range(num_vehs):
        start = np.array([-8, np.random.uniform(-1, 1), 0])
        end = np.array([8, start[1], 0])
        agents.append({"type": "vehicle", "path": [start + (end-start)*t for t in np.linspace(0, 1, 15)]})

    # 2. Slow-Motion Simulation Loop: Resilience & Betrayal Test
    leader_ped = agents[0]
    target_veh = agents[20] # First vehicle
    traitor_coords = None

    for step in range(15):
        # Time Dilation: x100 slow motion
        sim.step(dt=1.0, time_dilation=100.0)
        fusion.update_voxel_coherence()

        # Sense and React
        active_coords = []
        for agent in agents:
            pos = agent["path"][step]
            if agent["type"] == "pedestrian":
                vox = hsi.add_point(*pos, genome_update={'c': 0.2, 'e': 0.9, 'i': 0.8})
            else:
                vox = hsi.add_point(*pos, genome_update={'c': 0.9, 'e': 0.5, 'i': 0.9})

            active_coords.append(vox.coords)
            if vox.coords not in controllers:
                controllers[vox.coords] = MetasurfaceController(vox)

        # Metasurface Consensus & Tension
        latencies = []
        for coords in active_coords:
            voxel = hsi.voxels[coords]

            # Integrated Immune Feedback
            if voxel.is_quarantined:
                if coords in controllers:
                    controllers[coords].early_warning_pulse()

            ctrl = controllers[coords]
            nb_coords = hsi.get_neighbors(coords)
            nb_ctrls = [controllers[c] for c in nb_coords if c in controllers]
            target = {"radiative_cooling": True} if ctrl.voxel.genome.c > 0.7 else {"reflectivity": 1.0}
            ctrl.propose_state(nb_ctrls, target)
            latencies.append(ctrl.consensus_latency_ms)

        tau = sim.calculate_entanglement_tension(
            hsi.cartesian_to_hex(*leader_ped["path"][step]),
            hsi.cartesian_to_hex(*target_veh["path"][step])
        )
        avg_phi = sum(v.phi for v in hsi.voxels.values()) / len(hsi.voxels) if hsi.voxels else 0

        print(f"Step {step:2d}: Voxels={len(hsi.voxels):3d}, Φ_avg={avg_phi:.4f}, τ_leader={tau:.4f}")

        # Trigger Betrayal (Judas Protocol) at peak tension
        if step == 7:
            traitor_ped = agents[5]
            traitor_coords = hsi.cartesian_to_hex(*traitor_ped["path"][step])
            print(f"\n⚡ BREAKPOINT: Injecting Judas Protocol (Step 7, τ={tau:.4f})")
            sim.betrayal_protocol(traitor_coords)
            print("  >>> Group Resilience under test...")

        time.sleep(0.05)

    # 3. Auto-Healing & Reconciliation (Post-Crisis)
    print("\n🩹 CRITICAL POINT: Initiating Auto-Healing (Option 1: Grover Urbano)...")
    from arkhe.grover import GroverUrbano
    grover = GroverUrbano(hsi)

    # Identify affected voxels around the betrayal site
    affected_voxels = hsi.get_neighbors(traitor_coords)
    optimal_config = grover.search_optimal_config(affected_voxels, fitness_fn=lambda x: 1.0)
    grover.apply_healing(optimal_config)

    print("\n🤝 RECONCILIATION: Gradual reintegration of the dissident node...")
    for _ in range(3):
        sim.reconciliation_phase(traitor_coords)
        time.sleep(0.1)

    # 4. Materialization to Physical Metasurface
    print("\n🏗️ MATERIALIZATION (Option A): Tatuagem Permanente no Grafeno...")
    bias_map = sim.materialize_memory_to_bias(list(hsi.voxels.keys())[:10])

    # 5. Memory Analysis (Snapshot)
    print("\n❄️ Freezing HSI for Memory Analysis...")
    analyzer = MemoryAnalyzer(hsi)
    analyzer.generate_engram_report(baseline)

    print("\n🌿 Relaxation (Homeostasis) Phase...")
    for i in range(5):
        sim.relax(dt=1.0, b_reduction=0.1)
        avg_phi = sum(v.phi for v in hsi.voxels.values()) / len(hsi.voxels) if hsi.voxels else 0
        print(f"  Relaxation Step {i}: Coherence={avg_phi:.4f}")

    print("\n✅ Arkhe(n) Swarm Process Complete.")
    print("The city organism has evolved through collective interference.")

if __name__ == "__main__":
    main()
