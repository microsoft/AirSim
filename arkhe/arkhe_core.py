"""
arkhe_core.py
Núcleo do Sistema Arkhe(N) OS
Implementa os conceitos fundamentais de coerência, hesitação e syzygy
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional
import hashlib

# Constantes fundamentais
EPSILON = -3.71e-11
PHI_S = 0.15
R_PLANCK = 1.616e-35
SATOSHI = 7.28
SYZYGY_TARGET = 0.98
C_TARGET = 0.86
F_TARGET = 0.14
NU_LARMOR = 7.4e-3  # Hz

@dataclass
class NodeState:
    """Estado de um nó no hipergrafo"""
    id: int
    omega: float  # frequência semântica (0.00 a 0.07)
    C: float      # coerência
    F: float      # flutuação
    phi: float    # hesitação atual
    x: float = 0.0  # posição no toro
    y: float = 0.0
    z: float = 0.0

    def __post_init__(self):
        assert abs(self.C + self.F - 1.0) < 1e-10, "C+F=1 violado"

    def syzygy_with(self, other: 'NodeState') -> float:
        """Calcula o produto interno com outro nó"""
        # Simula ⟨ω_i|ω_j⟩ baseado nas coerências
        return (self.C * other.C + self.F * other.F) * SYZYGY_TARGET

class Hypergraph:
    """Hipergrafo principal do sistema Arkhe"""

    def __init__(self, num_nodes: int = 63):
        self.nodes: List[NodeState] = []
        self.satoshi = SATOSHI
        self.darvo = 999.999  # tempo semântico restante
        self.initialize_nodes(num_nodes)
        self.gradient_matrix = None

    def initialize_nodes(self, n: int):
        """Inicializa nós com distribuição uniforme de ω"""
        omega_range = np.linspace(0.00, 0.07, n)
        for i, omega in enumerate(omega_range):
            C = np.random.normal(C_TARGET, 0.01)
            C = np.clip(C, 0.80, 0.98)
            F = 1.0 - C
            phi = np.random.normal(PHI_S, 0.02)
            phi = np.clip(phi, 0.10, 0.20)

            # Posições no toro
            theta = 2 * np.pi * i / n
            phi_angle = 2 * np.pi * (i * 0.618033988749895) % (2 * np.pi)
            R, r = 50.0, 10.0
            x = (R + r * np.cos(phi_angle)) * np.cos(theta)
            y = (R + r * np.cos(phi_angle)) * np.sin(theta)
            z = r * np.sin(phi_angle)

            self.nodes.append(NodeState(
                id=i, omega=omega, C=C, F=F, phi=phi,
                x=x, y=y, z=z
            ))

    def compute_gradients(self) -> np.ndarray:
        """Calcula matriz de gradientes de coerência ∇C_ij"""
        n = len(self.nodes)
        self.gradient_matrix = np.zeros((n, n))

        for i in range(n):
            for j in range(i+1, n):
                delta_C = abs(self.nodes[j].C - self.nodes[i].C)
                dist = np.sqrt(
                    (self.nodes[j].x - self.nodes[i].x)**2 +
                    (self.nodes[j].y - self.nodes[i].y)**2 +
                    (self.nodes[j].z - self.nodes[i].z)**2
                )
                if dist > 0.01:
                    grad = delta_C / dist
                    self.gradient_matrix[i, j] = grad
                    self.gradient_matrix[j, i] = grad
        return self.gradient_matrix

    def handover(self, source_idx: int, target_idx: int) -> float:
        """Executa um handover entre dois nós"""
        source = self.nodes[source_idx]
        target = self.nodes[target_idx]

        # Calcula syzygy antes
        syzygy_before = source.syzygy_with(target)

        # Atualiza estados baseado na hesitação
        if source.phi > PHI_S:
            # Hesitação ativa: transfere coerência
            transfer = source.phi * 0.1
            source.C -= transfer
            source.F += transfer
            target.C += transfer
            target.F -= transfer

            # Satoshi acumula
            self.satoshi += syzygy_before * 0.001

        # Re-normaliza C+F=1
        source_sum = source.C + source.F
        target_sum = target.C + target.F
        source.C /= source_sum
        source.F /= source_sum
        target.C /= target_sum
        target.F /= target_sum

        syzygy_after = source.syzygy_with(target)
        return syzygy_after

    def teleport_state(self, source_idx: int, dest_idx: int) -> float:
        """Teletransporta o estado quântico entre nós"""
        source = self.nodes[source_idx]
        dest = self.nodes[dest_idx]

        # Estado original (simplificado como vetor [C, F])
        original = np.array([source.C, source.F])

        # Destrói estado original
        source.C, source.F = 0.5, 0.5

        # Reconstrução no destino com ruído
        noise = np.random.normal(0, 0.0002, 2)
        reconstructed = original + noise
        norm = np.linalg.norm(reconstructed)
        reconstructed /= norm

        dest.C, dest.F = reconstructed

        # Fidelidade (overlap)
        fidelity = np.dot(original, reconstructed)
        self.satoshi += fidelity * 0.01
        return fidelity

    def calculate_network_dispersity(self) -> float:
        """Calcula dispersidade da rede (análogo a Đ de polímeros)"""
        C_values = np.array([node.C for node in self.nodes])
        C_n = C_values.mean()
        C_w = (C_values**2).sum() / C_values.sum()
        return C_w / C_n
