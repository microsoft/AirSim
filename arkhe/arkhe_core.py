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
SATOSHI = 9.48  # Handover Γ₁₃₀ (Public Launch)
SYZYGY_TARGET = 1.000
C_TARGET = 0.86
F_TARGET = 0.14
NU_LARMOR = 0.0018  # GHz (Γ₁₃₀)

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
        return (self.C * other.C + self.F * other.F) * SYZYGY_TARGET

class Hypergraph:
    """Hipergrafo principal do sistema Arkhe"""

    def __init__(self, num_nodes: int = 12774):
        self.nodes: List[NodeState] = []
        self.satoshi = SATOSHI
        self.darvo = 1278.8  # Silêncio próprio Γ₁₃₀
        self.r_rh = 2.5e-8    # r/r_h (Γ₁₃₀)
        self.tunneling_prob = 1.000 # T_tunelamento (Γ₁₃₀)
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
        n = len(self.nodes)
        self.gradient_matrix = np.zeros((n, n))
        for i in range(n):
            for j in range(i+1, n):
                delta_C = abs(self.nodes[j].C - self.nodes[i].C)
                dist = np.sqrt((self.nodes[j].x - self.nodes[i].x)**2 + (self.nodes[j].y - self.nodes[i].y)**2 + (self.nodes[j].z - self.nodes[i].z)**2)
                if dist > 0.01:
                    grad = delta_C / dist
                    self.gradient_matrix[i, j] = grad
                    self.gradient_matrix[j, i] = grad
        return self.gradient_matrix

    def calculate_network_dispersity(self) -> float:
        C_values = np.array([node.C for node in self.nodes])
        C_n = C_values.mean()
        C_w = (C_values**2).sum() / C_values.sum()
        return C_w / C_n
