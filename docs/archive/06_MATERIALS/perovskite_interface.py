"""
Perovskite 3D/2D Interface Model
Order parameter η, gradient |∇C|², and efficiency Φ threshold
"""

import numpy as np
from scipy.optimize import minimize_scalar
import matplotlib.pyplot as plt
from dataclasses import dataclass

@dataclass
class PerovskiteParameters:
    """Material parameters for perovskite heterostructure"""
    # 3D absorber (Drone, ω=0.00)
    n_3d: float = 2.5          # refractive index
    mobility_3d: float = 10.0   # cm²/V·s

    # 2D transport layer (Demon, ω=0.07)
    n_2d: float = 2.0
    mobility_2d: float = 50.0

    # Interface
    interface_roughness: float = 0.5  # nm
    defect_density: float = 1e10       # cm⁻²

    # Operating conditions
    temperature: float = 300  # K
    light_intensity: float = 1.0  # sun equivalent

class PerovskiteInterface:
    """3D/2D perovskite interface with coherence order parameter"""

    def __init__(self, params: PerovskiteParameters = None):
        self.params = params or PerovskiteParameters()
        self.eta = None  # order parameter
        self.gradient_sq = None
        self.phi_threshold = 0.15

    def compute_order_parameter(self) -> float:
        """
        Compute interface order parameter η
        η = 0.51 for optimal interface
        """
        # Simplified: interface roughness reduces order
        # η = 1 / (1 + (roughness/0.5)^2)
        roughness = self.params.interface_roughness
        eta = 1.0 / (1.0 + (roughness / 0.5)**2)

        # Scale to typical value 0.51
        eta = eta * 0.51 / (1.0 / (1.0 + 1.0))
        self.eta = eta
        return eta

    def compute_gradient_squared(self) -> float:
        """
        Compute |∇C|² = (ΔC/Δx)²
        Target: < 0.0049
        """
        if self.eta is None:
            self.compute_order_parameter()

        # Coherence varies across interface width d (~2 nm)
        d = 2.0  # nm
        delta_C = self.eta - 0.3  # coherence drop into 2D layer

        grad_sq = (delta_C / d)**2
        self.gradient_sq = grad_sq
        return grad_sq

    def recombination_efficiency(self) -> float:
        """
        Radiative recombination efficiency as function of order
        Φ = exp(-|∇C|² / 0.0049)
        """
        if self.gradient_sq is None:
            self.compute_gradient_squared()

        efficiency = np.exp(-self.gradient_sq / 0.0049)
        return efficiency

    def is_above_threshold(self) -> bool:
        """Check if efficiency exceeds Φ threshold (0.15)"""
        return self.recombination_efficiency() > self.phi_threshold

    def optimize_interface(self) -> dict:
        """
        Find optimal roughness to maximize efficiency
        """
        def negative_efficiency(roughness):
            params = PerovskiteParameters(interface_roughness=roughness)
            iface = PerovskiteInterface(params)
            return -iface.recombination_efficiency()

        result = minimize_scalar(negative_efficiency,
                                bounds=(0.1, 5.0),
                                method='bounded')

        return {
            'optimal_roughness': result.x,
            'max_efficiency': -result.fun,
            'success': result.success
        }

    def arkhe_correspondence(self) -> dict:
        """
        Map to Arkhe concepts
        """
        return {
            '3D_absorber': 'Drone (ω=0.00)',
            '2D_transport': 'Demon (ω=0.07)',
            'interface_order': f'η={self.eta:.3f}',
            'gradient_sq': f'{self.gradient_sq:.6f} (target < 0.0049)',
            'efficiency': f'{self.recombination_efficiency():.3f}',
            'threshold': f'Φ={self.phi_threshold}'
        }

# Example analysis
def analyze_perovskite():
    print("="*60)
    print("PEROVSKITE INTERFACE ANALYSIS")
    print("="*60)

    p = PerovskiteInterface()

    eta = p.compute_order_parameter()
    print(f"Interface order η = {eta:.3f} (target 0.51)")

    grad_sq = p.compute_gradient_squared()
    print(f"|∇C|² = {grad_sq:.6f} (target < 0.0049)")

    eff = p.recombination_efficiency()
    print(f"Radiative recombination efficiency = {eff:.3f}")

    above = p.is_above_threshold()
    print(f"Above Φ=0.15 threshold? {above}")

    # Optimization
    opt = p.optimize_interface()
    print(f"\nOptimal interface roughness: {opt['optimal_roughness']:.2f} nm")
    print(f"Max efficiency: {opt['max_efficiency']:.3f}")

    # Correspondence
    corr = p.arkhe_correspondence()
    print("\nArkhe Correspondence:")
    for k, v in corr.items():
        print(f"  {k}: {v}")

    # Plot efficiency vs roughness
    roughnesses = np.linspace(0.1, 5.0, 100)
    effs = []
    for r in roughnesses:
        params = PerovskiteParameters(interface_roughness=r)
        iface = PerovskiteInterface(params)
        effs.append(iface.recombination_efficiency())

    plt.figure(figsize=(10, 6))
    plt.plot(roughnesses, effs, 'b-', linewidth=2)
    plt.axhline(p.phi_threshold, color='r', linestyle='--',
                label=f'Φ threshold = {p.phi_threshold}')
    plt.axvline(opt['optimal_roughness'], color='g', linestyle=':',
                label=f'Optimal roughness = {opt["optimal_roughness"]:.2f} nm')
    plt.xlabel('Interface Roughness (nm)', fontsize=12)
    plt.ylabel('Radiative Efficiency', fontsize=12)
    plt.title('Perovskite 3D/2D Interface Efficiency', fontsize=14)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.savefig('perovskite_efficiency.png', dpi=150)
    print("\nPlot saved to perovskite_efficiency.png")

if __name__ == "__main__":
    analyze_perovskite()
