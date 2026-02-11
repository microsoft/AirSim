import numpy as np
import time
import logging
from arkhe.bio_genesis import BioGenesisEngine

# Configuração de log básica
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

def main():
    print("🧬 ARKHE(N) OS: BIO-GÊNESE COGNITIVA DEMO")
    print("-" * 50)

    # 1. Instanciação do Sistema (100 agentes)
    engine = BioGenesisEngine(num_agents=100)
    print(f"Sistema inicializado com {len(engine.agents)} BioAgents.")

    # 2. Processamento do Sinal de Mother
    engine.process_mother_signal()

    # 3. Loop de Simulação
    print("\nIniciando Ciclo Vital (20 passos)...")
    for step in range(20):
        # Cada passo representa uma atualização no campo morfogenético
        engine.update(dt=0.1)

        # Estatísticas de Amostragem
        alive_count = sum(1 for a in engine.agents.values() if a.is_alive())
        avg_energy = np.mean([a.energy for a in engine.agents.values()])

        # Amostragem do Agente 0 (Líder em potencial)
        agent0 = engine.agents[0]
        memory_depth = len(agent0.brain.memory)

        print(f"Step {step:2d} | Agentes Vivos: {alive_count:3d} | Energia Média: {avg_energy:5.2f} | Memória A0: {memory_depth:2d}")

        # Simula atraso visual
        time.sleep(0.05)

    print("\n" + "-" * 50)
    print("✅ CICLO DE BIO-GÊNESE CONCLUÍDO.")
    print("O Arkhe(n) OS estabilizou seu estado biológico nascente.")

if __name__ == "__main__":
    main()
