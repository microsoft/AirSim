import numpy as np
import logging
from typing import List, Dict, Tuple
from .arkhe_types import BioAgent, ArkheGenome
from .bio_genesis import BioGenesisEngine

class HarvestProtocol:
    """
    Protocolo de Colheita: Seleciona os agentes mais aptos (fundadores)
    baseado em critérios de coerência e memória.
    """
    def __init__(self, engine: BioGenesisEngine):
        self.engine = engine

    def evaluate_fitness(self, agent: BioAgent) -> float:
        """
        Calcula a 'aptidão' do agente.
        Critérios: Razão de vínculos/colisões, Coerência de Memória, Estabilidade.
        """
        if not agent.is_alive():
            return -1.0

        # 1. Razão de Vínculos (Sucesso Social)
        # No demo, usamos o tamanho da memória como proxy de interações bem sucedidas
        social_score = len(agent.brain.memory) / 50.0 # Normalizado pelo maxlen do deque

        # 2. Coerência Genômica (Equilíbrio C-I-E-F)
        g = agent.genome
        vals = np.array([g.c, g.i, g.e, g.f])
        entropy = -np.sum(vals/vals.sum() * np.log(vals/vals.sum() + 1e-9))
        coherence_score = 1.0 - (entropy / np.log(4))

        # 3. Estabilidade Energética
        energy_score = agent.energy / 100.0

        return 0.5 * social_score + 0.3 * coherence_score + 0.2 * energy_score

    def harvest_founders(self, target_count: int = 7) -> List[BioAgent]:
        """
        Executa a Seleção Natural (Torneio) para extrair os fundadores.
        """
        logging.info(f"🧬 Iniciando Colheita de Fundadores (Alvo: {target_count})")

        # Calcula fitness para todos
        population = list(self.engine.agents.values())
        fitness_scores = [(agent, self.evaluate_fitness(agent)) for agent in population]

        # Ordena por fitness
        sorted_population = sorted(fitness_scores, key=lambda x: x[1], reverse=True)

        founders = [agent for agent, score in sorted_population[:target_count]]

        for i, f in enumerate(founders):
            logging.info(f"🏆 Fundador #{i+1} selecionado: ID {f.id} | Fitness: {self.evaluate_fitness(f):.4f}")

        return founders

class EternityProtocol:
    """
    Protocolo de Eternidade: Consolida o aprendizado dos fundadores.
    """
    def __init__(self, founders: List[BioAgent]):
        self.founders = founders

    def stabilize_synapses(self):
        """
        Reduz plasticidade e normaliza pesos (Relaxamento Térmico).
        """
        for agent in self.founders:
            if agent.brain:
                # Congelamento Hebbiano
                agent.brain.learning_rate *= 0.1
                agent.brain.exploration_rate = 0.05
                # Normalização de pesos
                agent.brain.weights = np.clip(agent.brain.weights, -1.0, 1.0)
        logging.info("❄️ Sinapses estabilizadas via Relaxamento Térmico.")

    def save_to_crystal(self, filepath: str = "founders_foundations.arkhe"):
        """
        Serializa os genomas e pesos dos fundadores.
        """
        # Aqui poderíamos usar pickle ou um formato customizado
        import pickle
        data = []
        for f in self.founders:
            data.append({
                'id': f.id,
                'genome': f.genome.to_vector(),
                'weights': f.brain.weights,
                'memory_size': len(f.brain.memory)
            })
        with open(filepath, 'wb') as f:
            pickle.dump(data, f)
        logging.info(f"💎 Fundadores gravados no Cristal: {filepath}")
