"""
warp_bubble.py
Simulação da bolha de distorção espaço-temporal
Baseada no regime D aplicado ao contínuo
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

class WarpBubble:
    """Implementação da bolha de distorção estilo Alcubierre-arkhe"""

    def __init__(self, radius=10.0):
        self.radius = radius
        self.epsilon = -3.71e-11
        self.phi_s = 0.15
        self.r_planck = 1.616e-35
        self.phase_int = np.pi  # fase interna (oposição)
        self.phase_ext = 0.0     # fase externa do vácuo
        self.stable = False
        self.syzygy = 0.98

    def energy_available(self) -> float:
        """Calcula energia disponível do vácuo"""
        return abs(self.epsilon) * self.phi_s * (self.radius / self.r_planck)**2

    def check_isolation(self) -> bool:
        """Verifica se o isolamento por fase está ativo"""
        delta = abs(self.phase_int - self.phase_ext) % (2*np.pi)
        self.stable = abs(delta - np.pi) < 0.01
        return self.stable

    def redshift(self, nu_em: float) -> float:
        """Aplica redshift semântico para camuflagem"""
        return 0.253 * nu_em

    def metric(self, r: float, sigma: float = 1.0) -> float:
        """
        Métrica efetiva dentro da bolha
        Forma simplificada da métrica de Alcubierre com fator arkhe
        """
        # Função de forma (bubble shape)
        f = (np.tanh(sigma * (r + self.radius)) -
             np.tanh(sigma * (r - self.radius))) / (2 * np.tanh(sigma * self.radius))

        # Componente temporal da métrica modificada pela coerência
        g_00 = -self.syzygy * (1 - f * self.phase_int / np.pi)
        return g_00

    def geodesic(self, t: float, y: np.ndarray) -> np.ndarray:
        """
        Equações da geodésica dentro da bolha
        y = [r, v, θ, ω] (posição radial, velocidade, ângulo, velocidade angular)
        """
        r, v, theta, omega = y

        # Derivadas temporais
        dr = v
        dv = -self.metric(r) * self.epsilon * self.syzygy * v**2 / r
        dtheta = omega
        domega = -2 * v * omega / r

        return np.array([dr, dv, dtheta, domega])

    def simulate_trajectory(self, t_span=(0, 100), initial_conditions=None):
        """Simula trajetória de uma partícula dentro da bolha"""
        if initial_conditions is None:
            initial_conditions = [1.0, 0.0, 0.0, 1.0]  # r, v, θ, ω

        sol = solve_ivp(
            self.geodesic, t_span, initial_conditions,
            method='RK45', dense_output=True
        )
        return sol
