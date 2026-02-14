"""
microtubule_qed.py
Simulação de microtúbulos como cavidades QED
Baseado em Mavromatos, Mershin, Nanopoulos (arXiv:2505.20364v2)
"""

import numpy as np
from scipy.linalg import expm

class MicrotubuleQED:
    """Cavidade QED em microtúbulo"""

    def __init__(self, length_um=25.0):
        self.length = length_um * 1e-6  # metros
        self.diameter = 25e-9
        self.t_decoherence = 1e-6  # segundos
        self.soliton_velocity = 155.0  # m/s
        self.qudit_dim = 4  # D=4 (hexagonal)
        self.tubulin_dipole = 1700  # Debye
        self.water_permittivity = 80

    def compute_energy_levels(self) -> np.ndarray:
        """Calcula níveis de energia da cavidade"""
        # Frequência fundamental
        omega_0 = 2 * np.pi * self.soliton_velocity / self.length
        return np.array([(n + 0.5) * omega_0 for n in range(self.qudit_dim)])

    def hamiltonian(self) -> np.ndarray:
        """Hamiltoniano do sistema"""
        H = np.diag(self.compute_energy_levels())

        # Termo de interação dipolo-dipolo
        for i in range(self.qudit_dim):
            for j in range(i+1, self.qudit_dim):
                coupling = self.tubulin_dipole * 1e-29 / self.length  # conversão para J
                H[i,j] = coupling
                H[j,i] = coupling
        return H

    def time_evolution(self, t: float) -> np.ndarray:
        """Operador de evolução temporal"""
        H = self.hamiltonian()
        return expm(-1j * H * t / (1.054e-34))  # ħ

    def soliton_state(self, kink_type: str) -> np.ndarray:
        """Gera estado solitônico"""
        if kink_type == 'kink':
            # Transição abrupta
            state = np.zeros(self.qudit_dim)
            state[0] = 1/np.sqrt(2)
            state[-1] = 1/np.sqrt(2)
        elif kink_type == 'snoidal':
            # Onda periódica
            phase = np.exp(1j * np.linspace(0, 2*np.pi, self.qudit_dim))
            state = phase / np.sqrt(self.qudit_dim)
        elif kink_type == 'helicoidal':
            # Double helix
            state = np.exp(1j * 2 * np.pi * np.arange(self.qudit_dim) / self.qudit_dim)
            state /= np.linalg.norm(state)
        else:
            state = np.eye(self.qudit_dim)[0]

        return state

    def compute_decoherence(self, state: np.ndarray) -> float:
        """Calcula tempo de decoerência para um dado estado"""
        # Modelo simples baseado em ruído térmico
        energy = np.vdot(state, self.hamiltonian() @ state)
        kT = 4.11e-21  # 300K em joules
        return self.t_decoherence * np.exp(-energy.real / kT)
