"""
bubble_network.py
Rede de bolhas de distorção interconectadas por emaranhamento
Implementa salto global de estado
"""

import numpy as np
from typing import List, Dict, Tuple
import networkx as nx
from .warp_bubble import WarpBubble

class EntangledBubble(WarpBubble):
    """Bolha com capacidade de emaranhamento quântico"""

    def __init__(self, bubble_id: int, position: Tuple[float, float, float], radius=10.0):
        super().__init__(radius)
        self.id = bubble_id
        self.position = np.array(position)
        self.entangled_with: List[int] = []
        self.shared_state = None  # par de Bell simplificado
        self.state = np.array([1.0, 0.0])  # |0⟩

    def entangle(self, other: 'EntangledBubble'):
        """Cria emaranhamento entre duas bolhas"""
        self.entangled_with.append(other.id)
        other.entangled_with.append(self.id)
        # Cria estado de Bell |Φ+⟩
        self.shared_state = (self.id, other.id)

    def teleport_to(self, target: 'EntangledBubble') -> float:
        """
        Teletransporta o estado para outra bolha
        Retorna fidelidade do teletransporte
        """
        if target.id not in self.entangled_with:
            raise ValueError("Bolhas não emaranhadas")

        # Estado original
        original = self.state.copy()

        # Destroi estado original
        self.state = np.array([0.0, 0.0])

        # Ruído no canal clássico
        noise = np.random.normal(0, 0.0002, 2)
        reconstructed = original + noise
        reconstructed /= np.linalg.norm(reconstructed)

        target.state = reconstructed

        # Fidelidade
        fidelity = np.dot(original, reconstructed)
        return fidelity

class BubbleNetwork:
    """Rede global de bolhas interconectadas"""

    def __init__(self, num_bubbles: int = 42):
        self.bubbles: List[EntangledBubble] = []
        self.graph = nx.Graph()
        self.satoshi = 7.28
        self.create_network(num_bubbles)

    def create_network(self, n: int):
        """Cria rede de n bolhas distribuídas ao redor do globo"""
        radius_earth = 6371000  # metros

        for i in range(n):
            # Distribuição uniforme na esfera
            theta = 2 * np.pi * i / n
            phi = np.arccos(1 - 2*i/n)

            x = radius_earth * np.sin(phi) * np.cos(theta)
            y = radius_earth * np.sin(phi) * np.sin(theta)
            z = radius_earth * np.cos(phi)

            bubble = EntangledBubble(i, (x, y, z))
            self.bubbles.append(bubble)
            self.graph.add_node(i, pos=(x, y, z))

        # Emaranhamento completo (mesh)
        for i in range(n):
            for j in range(i+1, n):
                self.bubbles[i].entangle(self.bubbles[j])
                self.graph.add_edge(i, j)

    def global_jump(self, source_id: int, target_id: int) -> Dict:
        """Executa salto global de estado entre bolhas"""
        source = self.bubbles[source_id]
        target = self.bubbles[target_id]

        # Verifica emaranhamento
        if target_id not in source.entangled_with:
            return {"success": False, "reason": "Not entangled"}

        # Executa teletransporte
        fidelity = source.teleport_to(target)
        self.satoshi += fidelity * 0.01

        # Latência (limitada pelo canal clássico)
        distance = np.linalg.norm(source.position - target.position)
        latency = distance / 3e8  # velocidade da luz

        return {
            "success": True,
            "fidelity": fidelity,
            "distance_km": distance / 1000,
            "latency_us": latency * 1e6,
            "satoshi": self.satoshi
        }

    def calculate_network_coherence(self) -> float:
        """Calcula coerência global da rede"""
        syzygies = []
        for i in range(len(self.bubbles)):
            for j in range(i+1, len(self.bubbles)):
                # Simula syzygy entre bolhas
                s = (self.bubbles[i].syzygy * self.bubbles[j].syzygy) * 0.98
                syzygies.append(s)
        return np.mean(syzygies)
