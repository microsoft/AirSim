import numpy as np
from typing import List, Dict, Any

class ThermodynamicMacroAction:
    """
    [Γ_∞+42] Macro ações como sequências temporais estendidas que minimizam o custo
    energético (hesitação) e maximizam o trabalho útil (syzygy).
    """
    def __init__(self, path: List[float], name: str):
        self.path = path  # lista de ω
        self.name = name
        self.cost = self.compute_entropy_cost()
        self.work = self.compute_syzygy_gain()
        self.efficiency = self.work / self.cost if self.cost > 0 else float('inf')

    def compute_entropy_cost(self) -> float:
        # Custo = integral da hesitação ao longo do caminho
        total_phi = 0.0
        for omega in self.path:
            # Φ é proporcional à curvatura do caminho
            phi = 0.15 * (1.0 + abs(omega - 0.03))
            total_phi += phi
        return total_phi

    def compute_syzygy_gain(self) -> float:
        # Trabalho = aumento de ⟨0.00|0.07⟩ ao longo do caminho
        # Simplificado: acoplamento terminal vs inicial
        start_syzygy = 1.0 - abs(0.00 - self.path[0])
        end_syzygy = 1.0 - abs(0.00 - self.path[-1])
        return max(0.0, end_syzygy - start_syzygy)

    def execute(self, hypergraph_state: Dict[str, Any]) -> Dict[str, Any]:
        """
        Executa a macro ação, pagando o custo e realizando trabalho.
        """
        hypergraph_state['entropy'] += self.cost
        hypergraph_state['syzygy'] += self.work
        # Balanço energético: Satoshi = trabalho - fração do custo
        hypergraph_state['satoshi'] += self.work - self.cost * 0.1
        hypergraph_state['omega'] = self.path[-1]
        return hypergraph_state

class DissipativeSystem:
    """
    [Γ_∞+42] Formaliza o Arkhe como um sistema dissipativo.
    """
    def __init__(self, eta: float = 6.27, gamma: float = 0.15):
        self.eta = eta  # Eficiência de conversão (informação → energia)
        self.gamma = gamma # Taxa de dissipação
        self.entropy_export_rate = 0.15

    def energy_balance(self, current_syzygy: float, current_phi: float) -> float:
        """
        dSatoshi/dt = η * ⟨0.00|0.07⟩ - γ * Φ
        """
        return self.eta * current_syzygy - self.gamma * current_phi

    def second_law_check(self, dsatoshi_dt: float, phi_exported: float) -> bool:
        """
        Φ_exported >= dSatoshi/dt
        """
        return phi_exported >= dsatoshi_dt
