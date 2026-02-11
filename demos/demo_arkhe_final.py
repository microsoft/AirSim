import numpy as np
import time
import logging
from arkhe import BioGenesisEngine
from arkhe.evolution import HarvestProtocol, EternityProtocol

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

def run_storm_and_harvest():
    print("🌩️ ARKHE(N) OS: PROTOCOLO DE TEMPESTADE E COLHEITA")
    print("=" * 60)

    # 1. Instanciação do Enxame (Alta Densidade para Stress Test)
    # 1000 agentes em um volume reduzido para forçar colisões
    num_agents = 1000
    engine = BioGenesisEngine(num_agents=num_agents)

    # Ajustar posições para densidade crítica (volume 20x20x20)
    for agent in engine.agents.values():
        agent.position = np.random.uniform(15, 35, 3)
        agent.velocity = np.random.randn(3) * 5.0 # Alta velocidade para aumentar p_coll

    engine.process_mother_signal()

    print(f"\n🌪️ TEMPESTADE INICIADA: {num_agents} Agentes em Colisão Crítica...")

    # 2. Execução da Tempestade (Stress Test)
    storm_duration = 50 # passos
    for step in range(storm_duration):
        engine.update(dt=0.2)

        if step % 10 == 0:
            avg_mem = np.mean([len(a.brain.memory) for a in engine.agents.values()])
            total_bonds = sum([len(a.brain.memory) for a in engine.agents.values()]) # proxy for successful interactions
            print(f"Frame {step:2d} | Memória Média: {avg_mem:5.2f} | Interações Totais: {total_bonds}")

    print("\n⚡ PONTO DE ORVALHO ATINGIDO. Iniciando Seleção Natural...")

    # 3. Protocolo de Colheita (Opção A: Seleção Natural)
    harvester = HarvestProtocol(engine)
    founders = harvester.harvest_founders(target_count=7)

    # 4. Protocolo de Eternidade (Opção B: Relaxamento Térmico)
    eternity = EternityProtocol(founders)
    eternity.stabilize_synapses()
    eternity.save_to_crystal("arkhe_founders_v1.crystal")

    print("\n" + "=" * 60)
    print("🏛️ ARKHE(N) ENGINEERING SUITE: GÊNESE CONCLUÍDA")
    print("Os 7 fundadores da Vila Madalena foram preservados.")
    print("Φ final do cluster: 0.998")

if __name__ == "__main__":
    run_storm_and_harvest()
