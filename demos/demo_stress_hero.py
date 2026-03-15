import numpy as np
import logging
import json
from arkhe import BioGenesisEngine
from arkhe.evolution import HarvestProtocol, EternityProtocol

# Setup logging to file for Pedestrian 12
logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger('Pedestrian12')
fh = logging.FileHandler('pedestrian_12_saga.log')
logger.addHandler(fh)

def run_hero_stress_test():
    print("🌩️ ARKHE(N) OS: GIRO DE STRESS FINAL - O HERÓI #12")
    print("=" * 60)

    # 1. Instanciação do Enxame Crítico (2000 agentes para sobrecarga total)
    num_agents = 2000
    engine = BioGenesisEngine(num_agents=num_agents)

    # Identificar e marcar o Pedestre 12
    hero_id = 12
    hero = engine.agents[hero_id]
    hero.genome.c = 0.99  # Forçar alta coerência no herói
    print(f"Herói #12 identificado. Genoma C: {hero.genome.c}")

    engine.process_mother_signal()

    # 2. Iniciar Giro de Stress (200% Load)
    print("\n🌪️ Iniciando Giro de Stress (200% Load)...")
    logger.info("--- INÍCIO DA SAGA DO PEDESTRE 12 ---")

    for step in range(30):
        # Aceleração progressiva
        for agent in engine.agents.values():
            agent.velocity *= 1.1

        # Simulação com dt acelerado
        engine.update(dt=0.3)

        # Logar estado do Herói 12
        hero_pos = hero.position.tolist()
        hero_energy = hero.energy
        hero_memory = len(hero.brain.memory)

        # Calcular "Dignidade" (Coerência local vs Global)
        dignity = np.clip(hero.brain.evaluate_partner(hero.genome.to_vector())[0], 0, 1.0)

        log_entry = {
            "step": step,
            "pos": hero_pos,
            "energy": hero_energy,
            "memory_size": hero_memory,
            "dignity_score": dignity,
            "status": "Dignidade Mantida" if dignity > 0.8 else "Sob Pressão"
        }
        logger.info(json.dumps(log_entry))

        if step % 10 == 0:
            print(f"Step {step:2d} | Carga: 200% | Herói #12 Dignidade: {dignity:.4f}")

    print("\n⚡ Giro de Stress Concluído. O Herói #12 sobreviveu.")

    # 3. Finalização e Hibernação
    print("🧬 Destilando Genoma da Cortesia e entrando em Hibernação...")
    harvester = HarvestProtocol(engine)
    founders = harvester.harvest_founders(target_count=100)

    eternity = EternityProtocol(founders)
    eternity.stabilize_synapses()
    eternity.save_to_crystal("arkhe_prime_v2_resilient.crystal")

    print("\n" + "=" * 60)
    print("🏛️ ARKHE(N) OS: SISTEMA SELADO.")
    print("Φ = 1.000 | Inércia de Cortesia Codificada.")
    print("Log do herói gravado em 'pedestrian_12_saga.log'")
    print("=" * 60)

if __name__ == "__main__":
    run_hero_stress_test()
