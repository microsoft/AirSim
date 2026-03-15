import numpy as np
from collections import deque
from typing import Optional, Tuple, List

class ConstraintLearner:
    """
    Cérebro do Agente: Aprende restrições e padrões temporais.
    Implementa memória episódica (deque) e aprendizado Hebbiano.
    """
    def __init__(self, agent_id: int, input_dim: int = 4, output_dim: int = 4, memory_size: int = 50):
        self.agent_id = agent_id
        # Inicializa pesos aleatórios baseados no genoma (4x4 por padrão para C-I-E-F)
        self.weights = np.random.randn(input_dim, output_dim) * 0.1
        self.memory = deque(maxlen=memory_size)
        self.learning_rate = 0.01
        self.exploration_rate = 0.1

    def evaluate_partner(self, partner_genome_vector: np.ndarray) -> Tuple[float, str]:
        """
        Avalia a viabilidade de conexão com um parceiro.
        Combina intuição (pesos) e experiência (memória).
        """
        # Predição baseada nos pesos (Intuição)
        intuition_score = np.tanh(np.dot(partner_genome_vector, self.weights).mean())

        # Reconhecimento de padrão na memória (Experiência)
        experience_score = 0.0
        if self.memory:
            # Similaridade simples com interações passadas
            similarities = []
            for state, _, reward in self.memory:
                sim = 1.0 - np.linalg.norm(state - partner_genome_vector)
                similarities.append((sim, reward))

            # Ponderação por similaridade
            weights = np.array([max(0, s[0]) for s in similarities])
            if weights.sum() > 0:
                experience_score = np.sum(weights * np.array([s[1] for s in similarities])) / weights.sum()

        # Fusão de Intuição e Experiência
        final_score = 0.7 * intuition_score + 0.3 * experience_score

        # Modulação por exploração (Curiosidade)
        if np.random.rand() < self.exploration_rate:
            final_score += np.random.uniform(-0.2, 0.2)
            reason = "Exploração"
        else:
            reason = "Consenso Cognitivo"

        return np.clip(final_score, -1.0, 1.0), reason

    def learn_from_experience(self, state: np.ndarray, action: np.ndarray, reward: float):
        """
        Atualiza pesos Hebbianos e armazena na memória episódica.
        """
        # Armazena a interação
        self.memory.append((state, action, reward))

        # Atualização Hebbiana: Δw = η * (Input * Erro/Reward)
        delta_w = self.learning_rate * np.outer(state, action) * reward
        self.weights += delta_w

        # Clipping para estabilidade
        self.weights = np.clip(self.weights, -2.0, 2.0)
