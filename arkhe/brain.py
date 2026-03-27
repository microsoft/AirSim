from collections import deque
import numpy as np
from typing import Optional, Tuple, List, Dict
from dataclasses import dataclass
from .arkhe_types import CIEF

@dataclass
class EpisodicTrace:
    """Registro completo de uma interação passada, com contexto temporal."""
    partner_genome: CIEF
    partner_id: int
    outcome: float          # delta energético real
    prediction: float       # score previsto antes da interação
    confidence: float       # |outcome| (surpresa)
    timestamp: float
    location: Tuple[float, float, float]

class ConstraintLearner:
    """
    ConstraintLearner: O cérebro do BioAgent.
    Reconhece padrões na 'cicatriz' do tempo usando memória episódica e pesos sinápticos.
    """
    def __init__(self, agent_id: int, genome_vector: Optional[np.ndarray] = None, memory_size: int = 50):
        self.agent_id = agent_id
        # Inicialização genômica: mistura de herança e aleatoriedade
        if genome_vector is not None:
            self.weights = genome_vector * 0.6 + np.random.randn(len(genome_vector)) * 0.4
        else:
            self.weights = np.random.randn(4) * 0.1

        self.weights = np.clip(self.weights, -2.5, 2.5)

        # Memória episódica – deque para Amnésia Homeostática Negativa
        self.episodic_memory: deque[EpisodicTrace] = deque(maxlen=memory_size)
        self.learning_rate = 0.01
        self.temporal_decay = 0.98
        self.exploration_rate = 0.1

    def remember(self, partner_id: int, partner_genome: CIEF, outcome: float, prediction: float, timestamp: float, position: Tuple[float, float, float]):
        """Armazena uma interação na memória episódica."""
        trace = EpisodicTrace(
            partner_genome=partner_genome,
            partner_id=partner_id,
            outcome=outcome,
            prediction=prediction,
            confidence=abs(outcome),
            timestamp=timestamp,
            location=position
        )
        self.episodic_memory.append(trace)

        # Hebbian Update: Δw = η * (Input * Erro_Predição)
        # Usando o genoma do parceiro como input
        partner_vec = partner_genome.to_array()
        self.weights += self.learning_rate * partner_vec * (outcome - prediction)

    def recall_similar(self, candidate_genome: CIEF, current_time: float) -> Tuple[float, float]:
        """
        Recupera memórias semelhantes e calcula um score ponderado.
        Retorna: (score_ponderado, entropia_das_memórias)
        """
        if not self.episodic_memory:
            return 0.0, 0.0

        weights = []
        scores = []
        candidate_vec = candidate_genome.to_array()

        for trace in self.episodic_memory:
            # 1. Similaridade genômica
            partner_vec = trace.partner_genome.to_array()
            sim = 1.0 - np.linalg.norm(candidate_vec - partner_vec) / 2.0
            sim = max(0.0, sim)

            # 2. Recência
            age = current_time - trace.timestamp
            recency = self.temporal_decay ** age

            # 3. Confiança (Surpresa)
            weight = sim * recency * trace.confidence
            weights.append(weight)
            scores.append(trace.outcome * 5.0)

        total = sum(weights) + 1e-8
        weights = [w / total for w in weights]

        episodic_score = sum(w * s for w, s in zip(weights, scores))
        entropy = -sum(w * np.log2(w + 1e-8) for w in weights)

        return np.clip(episodic_score, -1.0, 1.0), entropy

    def get_memory_entropy(self) -> float:
        """
        Calcula a entropia da distribuição de sucessos (outcomes) na memória.
        Baixa entropia = padrões cristalizados. Alta entropia = confusão/caos.
        """
        if not self.episodic_memory:
            return 0.0

        # Discretiza outcomes em bins para cálculo de entropia de Shannon
        outcomes = [trace.outcome for trace in self.episodic_memory]
        counts, _ = np.histogram(outcomes, bins=5, range=(-1, 1))
        probs = counts / (len(outcomes) + 1e-8)
        entropy = -sum(p * np.log2(p + 1e-8) for p in probs if p > 0)
        return float(entropy)

    def consolidate(self):
        """
        Consolida o aprendizado: 'Endurece' os pesos sinápticos baseando-se na memória episódica.
        Reduz a taxa de aprendizado e foca na exploração de padrões estáveis.
        """
        if not self.episodic_memory:
            return

        # Ajuste fino final baseado na média dos sucessos
        avg_outcome = np.mean([t.outcome for t in self.episodic_memory])
        self.weights *= (1.0 + avg_outcome * 0.1)
        self.learning_rate *= 0.1
        self.exploration_rate = 0.01
        print(f"   [BRAIN_{self.agent_id}] Aprendizado consolidado. Pesos estabilizados.")

    def evaluate_partner(self, partner_genome: CIEF, current_time: float = 0.0) -> Tuple[float, str]:
        """
        Combina intuição (pesos sinápticos) e memória (episódica).
        """
        partner_vec = partner_genome.to_array()
        semantic_score = np.tanh(np.dot(partner_vec, self.weights))

        episodic_score, entropy = self.recall_similar(partner_genome, current_time)

        # Fusão adaptativa: se há memórias claras (baixa entropia), dá mais peso
        memory_weight = 1.0 - np.clip(entropy / 5.0, 0.0, 0.8)

        if len(self.episodic_memory) > 5:
            final_score = memory_weight * episodic_score + (1 - memory_weight) * semantic_score
            reasoning = f"Memória({episodic_score:+.2f}) + Intuição({semantic_score:+.2f})"
        else:
            final_score = semantic_score
            reasoning = f"Intuição({semantic_score:+.2f})"

        return final_score, reasoning
