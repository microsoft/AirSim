import numpy as np
import logging
from typing import Dict, List, Optional
from .arkhe_types import BioAgent, ArkheGenome
from .cognition import ConstraintLearner
from .physics import SpatialHashGrid, calculate_collision_probability

class BioGenesisEngine:
    """
    Engine de Gênese: Gerencia BioAgents e o estado biológico do Arkhe(n) OS.
    """
    def __init__(self, num_agents: int = 100):
        self.agents: Dict[int, BioAgent] = {}
        self.spatial_hash = SpatialHashGrid(cell_size=4.0)
        self.simulation_time = 0.0
        self.next_id = 0
        self._initialize_population(num_agents)

    def _initialize_population(self, num_agents):
        for _ in range(num_agents):
            pos = np.random.uniform(0, 50, 3)
            # Genoma aleatório inicial
            genome = ArkheGenome(
                c=np.random.rand(),
                i=np.random.rand(),
                e=np.random.rand(),
                f=np.random.rand()
            )
            agent = BioAgent(id=self.next_id, position=pos, genome=genome)

            # Instancia o Cérebro
            brain = ConstraintLearner(agent_id=self.next_id)
            # Seed inicial aleatória a partir do genoma
            brain.weights = np.outer(genome.to_vector(), np.random.randn(4)) * 0.5
            agent.brain = brain

            self.agents[self.next_id] = agent
            self.next_id += 1

    def process_mother_signal(self):
        """
        Protocolo de Bio-Gênese Cognitiva: Ativa o estado biológico.
        """
        logging.info("🌱 Sinal primordial de Mother recebido.")
        # Ativação sistêmica: aumenta plasticidade inicial
        for agent in self.agents.values():
            if agent.brain:
                agent.brain.exploration_rate = 0.4
                agent.brain.learning_rate = 0.05
        logging.info(f"🚀 Arkhe(n) OS transicionado para estado biológico. {len(self.agents)} agentes ativos.")

    def update(self, dt: float):
        self.simulation_time += dt

        # 1. Atualiza Posições (Brownian Motion + Inércia Simples)
        for agent in self.agents.values():
            if not agent.is_alive(): continue
            agent.position += agent.velocity * dt + np.random.randn(3) * 0.1
            # Mantém no limite 0-50 (simulado)
            agent.position = np.clip(agent.position, 0, 50)

        # 2. Reconstrói Hash Grid
        self.spatial_hash.clear()
        for agent in self.agents.values():
            if agent.is_alive():
                self.spatial_hash.insert(agent.id, agent.position)

        # 3. Interações Sociais Priorizadas por Colisão
        self._process_interactions(dt)

    def _process_interactions(self, dt: float):
        processed_pairs = set()

        for agent in self.agents.values():
            if not agent.is_alive(): continue

            # Busca vizinhos eficientemente
            neighbor_ids = self.spatial_hash.query_radius(agent.position, radius=5.0)

            # Calcula prioridades (Probabilidade de Colisão)
            priorities = []
            for other_id in neighbor_ids:
                if other_id == agent.id: continue
                other = self.agents[other_id]
                if not other.is_alive(): continue

                p_coll = calculate_collision_probability(
                    agent.position, agent.velocity,
                    other.position, other.velocity, dt
                )
                priorities.append((other_id, p_coll))

            # Ordena por risco (Colisão)
            priorities.sort(key=lambda x: x[1], reverse=True)

            for other_id, p_coll in priorities:
                pair = tuple(sorted((agent.id, other_id)))
                if pair in processed_pairs: continue
                processed_pairs.add(pair)

                other = self.agents[other_id]

                # Avaliação Cognitiva
                score_a, reason_a = agent.brain.evaluate_partner(other.genome.to_vector())
                score_b, reason_b = other.brain.evaluate_partner(agent.genome.to_vector())

                # Consenso de conexão
                if (score_a + score_b) / 2.0 > 0.3 or p_coll > 0.8:
                    # Forma conexão / ajuste metabólico
                    reward = 0.1 * p_coll if p_coll > 0 else 0.05
                    agent.brain.learn_from_experience(other.genome.to_vector(), np.ones(4), reward)
                    other.brain.learn_from_experience(agent.genome.to_vector(), np.ones(4), reward)

                    # Consome energia pelo processamento
                    agent.energy -= 0.1
                    other.energy -= 0.1
