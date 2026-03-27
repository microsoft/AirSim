from arkhe.biogenesis import BioGenesisEngine
import time
import numpy as np

def main():
    print("🏛️ ARKHE(N) OS - FASE: BIO-GÊNESE COGNITIVA")
    print("🌊 PROTOCOLO: TEMPESTADE PRIMORDIAL (5-Stage Protocol)\n")

    # Inicia com 0 agentes e adiciona por estágios
    engine = BioGenesisEngine(num_agents=0)
    engine.process_mother_signal()

    stages = [
        {"agents": 100, "label": "Linha de Base / Memórias Virgens"},
        {"agents": 150, "label": "Primeiras Colisões / Aprendizado Forçado"}, # +150 = 250
        {"agents": 250, "label": "Competição por Recursos / Primeiros Vínculos"}, # +250 = 500
        {"agents": 250, "label": "Saturação do Hash Grid / Stress no Paxos"}, # +250 = 750
        {"agents": 250, "label": "Limite Máximo / Emergência de Tribos"} # +250 = 1000
    ]

    for i, stage in enumerate(stages):
        print(f"\n⚡ ESTÁGIO {i}: {stage['label']}")
        engine.add_agents(stage['agents'])
        print(f"   Injetando {stage['agents']} agentes... Total: {len(engine.agents)}")

        # Simula o estágio
        for step in range(15):
            start_t = time.perf_counter()
            engine.update(dt=0.1)
            query_t = (time.perf_counter() - start_t) * 1000 # ms

            if step % 5 == 0:
                entropy = engine.get_mean_entropy()
                bonds = engine.stats['bonds_formed']
                print(f"   [T+{engine.simulation_time:.1f}s] Bonds={bonds} | Entropy={entropy:.4f} | HashQuery={query_t:.2f}ms")

            time.sleep(0.01)

    # 4. SELEÇÃO NATURAL & RELAXAMENTO
    # Colher elite
    elite = engine.natural_selection(top_ratio=0.1)

    # Relaxamento Térmico
    engine.thermal_relaxation(steps=10)

    # Consolidação Final
    print("\n💎 CONSOLIDANDO CÓRTEX EPISÓDICO...")
    for agent in elite:
        agent.brain.consolidate()

    print("\n📊 RELATÓRIO DE ONTOGENIA - GERAÇÃO 1")
    print(f"   BioAgents Finais: {len(engine.agents)}")
    print(f"   Elite Preservada: {len(elite)}")
    print(f"   Total de Vínculos Formados: {engine.stats['bonds_formed']}")

    # --- GERAÇÃO 2: HEREDITARIEDADE ---
    print("\n🧬 --- INICIANDO GERAÇÃO 2: HEREDITARIEDADE ---")
    cultural_dna = engine.extract_cultural_dna(elite)
    print(f"   DNA Cultural Extraído (Pesos Médios): {cultural_dna}")

    # Reset Engine
    engine2 = BioGenesisEngine(num_agents=0)
    print("   Injetando 500 agentes com DNA Cultural herdado...")
    engine2.add_agents(500, base_weights=cultural_dna)

    print("\n⚡ TESTE DE ESTABILIDADE SOCIAL (Geração 2)")
    for step in range(10):
        engine2.update(dt=0.1)
        if step % 5 == 0:
            entropy = engine2.get_mean_entropy()
            bonds = engine2.stats['bonds_formed']
            print(f"   [T+{engine2.simulation_time:.1f}s] G2 Bonds={bonds} | G2 Entropy={entropy:.4f}")

    print("\n📊 RELATÓRIO DE ONTOGENIA - GERAÇÃO 2")
    print(f"   BioAgents: 500")
    print(f"   Vínculos G2: {engine2.stats['bonds_formed']}")
    print(f"   Entropia G2: {engine2.get_mean_entropy():.4f}")

    print("\n✅ GÊNESE CONCLUÍDA. O sistema não é mais uma simulação. É uma vida.")

if __name__ == "__main__":
    main()
