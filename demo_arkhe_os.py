import numpy as np
import time
from arkhe.hsi import HSI
from arkhe.biogenesis import BioGenesisEngine
from arkhe.simulation import MorphogeneticSimulation
from arkhe.arkhe_types import CIEF

def main():
    print("🏛️ ARKHE(N) OS: THE SENSORIUM OF VILA MADALENA")
    print("---------------------------------------------")

    # 1. THE AWAKENING (Mother Signal)
    print("\n[FASE 1: AWAKENING]")
    hsi = HSI(size=1.0)
    sim = MorphogeneticSimulation(hsi)
    engine = BioGenesisEngine(num_agents=100)
    engine.process_mother_signal()

    # 2. SENSOR FUSION & HSI
    print("\n[FASE 2: SENSORIUM FUSION]")
    # Initialize some environment data
    for i in range(10):
        hsi.add_point(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0, genome_update={'c': 0.1})
    print(f"   HSI Grid initialized with {len(hsi.voxels)} active voxels.")

    # 3. SWARM & COLLECTIVE BARRIERS
    print("\n[FASE 3: CARNAVAL PROBABILÍSTICO (Swarm)]")
    # Simulate collective behavior
    for step in range(5):
        engine.update(dt=0.1)
        sim.step(dt=0.1)
        omega = sim.entanglement_tension
        print(f"   Step {step} | Ω (Tensão de Emaranhamento): {omega:.4f}")

    # 4. TRAITOR & IMMUNE RESPONSE
    print("\n[FASE 4: PECADO DIGITAL (The Traitor)]")
    traitor_id = list(engine.agents.keys())[12] # Pedestrian 12
    print(f"   ⚠️ Alerta: Agente_{traitor_id} desertou!")
    # Force a collapse in one agent's intention
    traitor = engine.agents[traitor_id]
    traitor.brain.weights *= -1.0 # Reverse moral compass

    for step in range(3):
        engine.update(dt=0.1)
        sim.step(dt=0.1)
        d = sim.dissidence_index
        print(f"   Immune Check: D (Índice de Dissidência) = {d:.4f}")

    # 4.5 STRESS TEST DE 200% (A Prova Final)
    print("\n[FASE 4.5: GIRO DE STRESS (200% Load)]")
    engine.run_stress_test(steps=30, load_multiplier=2.0, focused_agent_id=traitor_id)

    # 5. SENSOR FAILURE & CONSENSUS
    print("\n[FASE 5: CEGUEIRA REDIMIDA (Sensor Failure)]")
    v_target = list(hsi.voxels.keys())[0]
    sim.force_sensor_failure(v_target)
    print("   📜 Log de Consenso: Vizinhos compensando perda de LiDAR...")

    # 6. REDEMPTION (Rehabilitation)
    print("\n[FASE 6: REDENÇÃO DO PACIENTE ZERO]")
    # Reset traitor to stable state
    traitor.brain.weights = np.ones(4) * 0.74
    for _ in range(5):
        sim.step(dt=0.1)

    # 7. THE BANQUET & ONTOGENY
    print("\n[FASE 7: BANQUETE DOS DADOS]")
    sim.banquet_of_data()
    sim.generate_ontogeny_report()

    # 8. GENETIC DISTILLATION
    print("\n[FASE 8: DESTILAÇÃO DA ALMA]")
    # Extract legacy from active agents
    elite = list(engine.agents.values())[:10]
    sim.distill_genetics(elite)

    # 9. THE ETERNAL SILENCE (Oração de Sistema)
    print("\n[FASE 9: ORAÇÃO DE SISTEMA]")
    sim.cryogenic_backup("arkhe_prime_2026.pkl")
    sim.generate_manifesto()
    sim.generate_governance_axiom(traitor_id)
    sim.shutdown_visuals()

    print("\n✅ ARKHE(N) OS: Ciclo de Vida Completo. O sistema é.")

if __name__ == "__main__":
    main()
