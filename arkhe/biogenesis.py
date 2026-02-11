import numpy as np
import time
from typing import Dict, List, Tuple, Optional
from .arkhe_types import CIEF, BioAgent
from .grid import SpatialHashGrid
from .brain import ConstraintLearner
from .telemetry import ArkheTelemetry

class BioGenesisEngine:
    """
    BioGenesisEngine: O coração do Arkhe(n) OS na fase de Bio-Gênese Cognitiva.
    """
    def __init__(self, num_agents: int = 100):
        self.agents: Dict[int, BioAgent] = {}
        self.spatial_hash = SpatialHashGrid(cell_size=3.0)
        self.telemetry = ArkheTelemetry()
        self.simulation_time = 0.0
        self.next_id = 1
        self.stats = {'births': 0, 'bonds_formed': 0}

        self._initialize_population(num_agents)

    def get_mean_entropy(self) -> float:
        """
        Calcula a entropia média das memórias de todos os agentes ativos.
        """
        entropies = [a.brain.get_memory_entropy() for a in self.agents.values() if a.brain and a.is_alive()]
        if not entropies: return 0.0
        return float(np.mean(entropies))

    def add_agents(self, num_agents: int, base_weights: Optional[np.ndarray] = None):
        """
        Injeta novos BioAgents no campo morfogenético.
        base_weights: Pesos hereditários (DNA Cultural).
        """
        for _ in range(num_agents):
            pos = np.random.uniform(-50, 50, 3)
            vel = np.random.uniform(-1, 1, 3)
            # Genoma CIEF aleatório para diversidade
            genome = CIEF(
                c=np.random.rand(),
                i=np.random.rand(),
                e=np.random.rand(),
                f=np.random.rand()
            )

            agent = BioAgent(self.next_id, pos, vel, genome)
            # Cérebro individual com diversidade genômica
            brain = ConstraintLearner(self.next_id, genome.to_array())

            # Hereditariedade: herda pesos culturais se disponíveis
            if base_weights is not None:
                # Mistura DNA Cultural com mutação local
                brain.weights = base_weights * 0.8 + np.random.randn(4) * 0.2
                brain.weights = np.clip(brain.weights, -2.5, 2.5)

            agent.set_brain(brain)

            self.agents[self.next_id] = agent
            self.next_id += 1
            self.stats['births'] += 1

    def _initialize_population(self, num_agents):
        self.add_agents(num_agents)

    def process_mother_signal(self, legacy_weights: Optional[np.ndarray] = None):
        """
        Recebe o sinal primordial e configura o estado inicial do sistema.
        legacy_weights: Se fornecido, o sinal carrega o 'DNA Cultural' (Patrimônio Ético).
        """
        if legacy_weights is not None:
            print("🌱 MOTHER SIGNAL RECEIVED: THE LEGACY (Destilação de Conhecimento)")
            # Injeta o legado em todos os novos agentes (Hereditariedade)
            for agent in self.agents.values():
                if agent.brain:
                    agent.brain.weights = legacy_weights * 0.9 + np.random.randn(4) * 0.1
                    agent.brain.exploration_rate = 0.1 # Lower exploration, higher instinct
        else:
            print("🌱 MOTHER SIGNAL RECEIVED: Gênese Primordial")
            for agent in self.agents.values():
                if agent.brain:
                    agent.brain.exploration_rate = 0.5

        self.telemetry.dispatch_channel_a({
            "timestamp": time.time(),
            "event": "biogenesis_awakening",
            "agent_count": len(self.agents),
            "is_legacy": legacy_weights is not None
        })
        print(f"✅ GÊNESE CONCLUÍDA – {len(self.agents)} AGENTES ATIVOS")

    def _collision_probability(self, agent_a: BioAgent, agent_b: BioAgent, dt: float) -> float:
        """Calcula probabilidade de encontro iminente."""
        pos_a, pos_b = agent_a.position, agent_b.position
        vel_a, vel_b = agent_a.velocity, agent_b.velocity

        dist = np.linalg.norm(pos_a - pos_b)
        if dist > 10.0: return 0.0

        # Velocidade relativa
        rel_vel = vel_a - vel_b
        dir_vec = (pos_b - pos_a) / (dist + 1e-6)
        approach_speed = max(0, -np.dot(rel_vel, dir_vec))

        if approach_speed <= 0: return 0.0

        time_to_contact = dist / approach_speed
        # Quanto menor o tempo, maior a probabilidade
        prob = np.exp(-time_to_contact / dt)
        return np.clip(prob, 0.0, 1.0)

    def update(self, dt: float = 0.1):
        self.simulation_time += dt

        # 1. Movimento e atualização de grade
        self.spatial_hash.clear()
        for agent in self.agents.values():
            if not agent.is_alive(): continue
            # Movimento Browniano simples + velocidade
            agent.position += agent.velocity * dt + np.random.randn(3) * 0.05
            self.spatial_hash.insert(agent)

        # 2. Interações Sociais O(N)
        processed_pairs = set()
        for agent in self.agents.values():
            if not agent.is_alive(): continue

            neighbors = self.spatial_hash.query_radius(agent.position, radius=5.0)
            for other in neighbors:
                if other.id <= agent.id or not other.is_alive(): continue

                pair = (agent.id, other.id)
                if pair in processed_pairs: continue
                processed_pairs.add(pair)

                # Cálculo de probabilidade de colisão (Desejo/Risco)
                prob_collision = self._collision_probability(agent, other, dt)

                # Avaliação cognitiva
                score_a, reason_a = agent.brain.evaluate_partner(other.genome, self.simulation_time)
                score_b, reason_b = other.brain.evaluate_partner(agent.genome, self.simulation_time)

                # Consenso ponderado pelo risco
                consensus = (score_a + score_b) / 2.0
                effective_score = consensus * (1.0 + prob_collision)

                if effective_score > 0.3:
                    # Formar vínculo (Bio-Gênese)
                    agent.connections[other.id] = effective_score
                    other.connections[agent.id] = effective_score
                    self.stats['bonds_formed'] += 1

                    # Recompensa baseada no sucesso da interação (simulado)
                    reward = 0.1 + prob_collision * 0.2
                    agent.brain.remember(other.id, other.genome, reward, score_a, self.simulation_time, agent.position)
                    other.brain.remember(agent.id, agent.genome, reward, score_b, self.simulation_time, other.position)

    def natural_selection(self, top_ratio=0.1):
        """
        Seleção Natural: Filtra e colhe os melhores genomas baseados em vínculos e energia.
        """
        print("\n🧬 INICIANDO SELEÇÃO NATURAL (Colher os melhores genomas)...")
        # Score = bonds * energy
        scored_agents = []
        for a in self.agents.values():
            score = len(a.connections) * a.energy
            scored_agents.append((score, a))

        scored_agents.sort(key=lambda x: x[0], reverse=True)
        top_count = int(len(self.agents) * top_ratio)
        elite = scored_agents[:top_count]

        print(f"   Elite de Gênese: {len(elite)} agentes selecionados.")
        for i, (score, agent) in enumerate(elite[:5]):
            print(f"   Top {i}: Agent_{agent.id} | Score={score:.2f} | Genome={agent.genome}")

        return [a for s, a in elite]

    def extract_cultural_dna(self, agents: List[BioAgent]) -> np.ndarray:
        """
        Extrai a média dos pesos sinápticos de um grupo de agentes (a 'Cultura').
        """
        weights = [a.brain.weights for a in agents if a.brain]
        if not weights: return np.zeros(4)
        return np.mean(weights, axis=0)

    def thermal_relaxation(self, steps=20):
        """
        Relaxamento Térmico: Reduz atividade e consolida aprendizado.
        """
        print("\n🧊 INICIANDO RELAXAMENTO TÉRMICO (Consolidar aprendizado)...")
        for i in range(steps):
            # Gradual reduction of learning and exploration
            factor = 1.0 - (i / steps)
            for a in self.agents.values():
                if a.brain:
                    a.brain.learning_rate *= 0.9
                    a.brain.exploration_rate *= 0.9

            self.update(dt=0.05) # Slower time steps
            if i % 5 == 0:
                print(f"   Relaxation Step {i}: Coerência Global melhorando...")

    def run_stress_test(self, steps=50, load_multiplier=1.0, focused_agent_id=None):
        print(f"\n🌪️ INICIANDO STRESS TEST DE DENSIDADE (Load={load_multiplier}x)...")
        if load_multiplier > 1.0:
            extra_agents = int(len(self.agents) * (load_multiplier - 1.0))
            print(f"   Injetando sobrecarga de {extra_agents} agentes...")
            self.add_agents(extra_agents)

        start_t = time.time()
        for i in range(steps):
            self.update(dt=0.1)

            # Focused log for the 'Hero'
            if focused_agent_id and focused_agent_id in self.agents:
                agent = self.agents[focused_agent_id]
                if i % 10 == 0:
                    print(f"   [HERO_LOG] Agente_{focused_agent_id} | Pos={agent.position} | Bonds={len(agent.connections)}")

            if i % 10 == 0:
                print(f"   Step {i}: Total Bonds={self.stats['bonds_formed']} | Entropy={self.get_mean_entropy():.4f}")
        end_t = time.time()
        print(f"🏁 STRESS TEST CONCLUÍDO. Tempo: {end_t - start_t:.2f}s")
