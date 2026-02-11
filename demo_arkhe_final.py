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
    print("🏛️ ARKHE(N) ENGINEERING SUITE - FINAL OMNIMODAL DEMO")

    hsi = HSI(size=0.5)
    fusion = FusionEngine(hsi)
    sim = MorphogeneticSimulation(hsi, learning_rate=0.05)
    baseline = HSISnapshot(hsi)
    controllers = {}

    print("\n🎭🎭 Simulating 'Carnaval Quântico': The Complete Cycle")

    # 1. Setup Agents
    num_peds = 20
    num_vehs = 10
    agents = []
    for i in range(num_peds):
        start = np.array([np.random.uniform(-2, 2), -5, 0])
        end = np.array([start[0], 5, 0])
        agents.append({"id": f"P{i}", "type": "pedestrian", "path": [start + (end-start)*t for t in np.linspace(0, 1, 20)]})
    for i in range(num_vehs):
        start = np.array([-8, np.random.uniform(-1, 1), 0])
        end = np.array([8, start[1], 0])
        agents.append({"id": f"V{i}", "type": "vehicle", "path": [start + (end-start)*t for t in np.linspace(0, 1, 20)]})

    # Pedestrian 12 (The Patient Zero)
    p12_id = agents[12]["id"]
    p12_traitor_triggered = False

    # 2. Simulation Loop
    for step in range(20):
        sim.step(dt=1.0)
        fusion.update_voxel_coherence()

        active_coords = []
        for agent in agents:
            pos = agent["path"][step]
            vox = hsi.get_voxel(hsi.cartesian_to_hex(*pos))

            # Scenario Injections
            if agent["id"] == "V0" and step >= 10:
                vox.sensor_health = 0.0 # Induce Blindness

            if agent["id"] == p12_id and not p12_traitor_triggered and step == 5:
                sim.betrayal_protocol(vox.coords)
                p12_traitor_triggered = True

            if agent["type"] == "pedestrian":
                fusion.fuse_lidar(np.array([pos])) # Simplified
                # Projecting some 'information' for pedestrians
                vox.genome.i += 0.05
            else:
                # Vehicles: High physicality
                vox.genome.c += 0.9
                # If blind, use neighbors (Cognitive Prosthesis is inside fuse_lidar)
                fusion.fuse_lidar(np.array([pos]))

            active_coords.append(vox.coords)
            if vox.coords not in controllers:
                controllers[vox.coords] = MetasurfaceController(vox)

        # Consensus and Rehabilitation
        for coords, vox in hsi.voxels.items():
            if vox.rehabilitation_index > 0 or p12_traitor_triggered:
                 # Reconciliation is slow
                 sim.reconciliation_phase(coords, recovery_rate=0.02)

        # Telemetry
        avg_phi = sum(v.phi for v in hsi.voxels.values()) / len(hsi.voxels)
        v0_vox = hsi.voxels.get(hsi.cartesian_to_hex(*agents[20]["path"][step]))
        p12_vox = hsi.voxels.get(hsi.cartesian_to_hex(*agents[12]["path"][step]))

        status_v0 = f"V0_Health={v0_vox.sensor_health:.1f}" if v0_vox else ""
        status_p12 = f"P12_Rehab={p12_vox.rehabilitation_index:.2f}" if p12_vox else ""

        print(f"Step {step:2d} | Φ_avg={avg_phi:.4f} | {status_v0} | {status_p12}")
        time.sleep(0.05)

    # 3. Final Reconciliation and Materialization
    print("\n🤝 Finalizing Reconciliation for all agents...")
    for _ in range(10): sim.reconciliation_phase(hsi.cartesian_to_hex(*agents[12]["path"][-1]))

    print("\n🏗️  Materializing Final Memory to Physical Metasurface...")
    sim.materialize_memory_to_bias(list(hsi.voxels.keys()))

    print("\n✅ Arkhe(n) OS: Sensorium Cycle Complete.")
    print("Vila Madalena is now a coherent, experienced, and physical organism.")

if __name__ == "__main__":
    main()
