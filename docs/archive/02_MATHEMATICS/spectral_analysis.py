"""
Spectral Analysis for C+F=1 Conservation
Computes coherence and fluctuation from time series data
"""

import numpy as np
from scipy import signal
from scipy.fft import fft, fftfreq
import matplotlib.pyplot as plt

class CoherenceAnalyzer:
    """Analyze coherence and fluctuation in time series"""

    def __init__(self, fs: float = 1000.0):
        """
        Args:
            fs: Sampling frequency (Hz)
        """
        self.fs = fs

    def compute_coherence(self, x1: np.ndarray, x2: np.ndarray,
                         nperseg: int = 256) -> tuple:
        """
        Compute magnitude-squared coherence between two signals

        Args:
            x1, x2: Input signals
            nperseg: Length of each segment for Welch's method

        Returns:
            f: Frequency array
            Cxy: Coherence array
            Fxy: Fluctuation array (1 - Cxy)
        """
        # Compute cross-spectral density
        f, Gxy = signal.csd(x1, x2, self.fs, nperseg=nperseg)

        # Compute auto-spectral densities
        _, Gxx = signal.welch(x1, self.fs, nperseg=nperseg)
        _, Gyy = signal.welch(x2, self.fs, nperseg=nperseg)

        # Magnitude-squared coherence
        Cxy = np.abs(Gxy)**2 / (Gxx * Gyy)

        # Fluctuation (complement)
        Fxy = 1.0 - Cxy

        return f, Cxy, Fxy

    def verify_conservation(self, Cxy: np.ndarray, Fxy: np.ndarray,
                          tol: float = 1e-10) -> bool:
        """Verify C + F = 1 within tolerance"""
        conservation = Cxy + Fxy
        return np.allclose(conservation, 1.0, atol=tol)

    def mutual_information_lower_bound(self, f: np.ndarray,
                                      Fxy: np.ndarray,
                                      fc: float) -> float:
        """
        Compute lower bound on mutual information

        I_LB = -∫₀^fc log₂(F(f)) df

        Args:
            f: Frequency array
            Fxy: Fluctuation spectrum
            fc: Cutoff frequency

        Returns:
            I_LB: Lower bound on mutual information (bits)
        """
        # Select frequencies up to cutoff
        mask = f <= fc
        f_sel = f[mask]
        F_sel = Fxy[mask]

        # Avoid log(0)
        F_sel = np.maximum(F_sel, 1e-10)

        # Integrate -log₂(F)
        integrand = -np.log2(F_sel)
        I_LB = np.trapz(integrand, f_sel)

        return I_LB

    def find_operational_point(self, Cxy: np.ndarray) -> dict:
        """
        Find frequency where coherence is closest to 0.86

        Returns:
            Dictionary with operational point info
        """
        target_C = 0.86
        idx = np.argmin(np.abs(Cxy - target_C))

        return {
            'index': idx,
            'C_actual': Cxy[idx],
            'F_actual': 1.0 - Cxy[idx],
            'deviation': np.abs(Cxy[idx] - target_C)
        }

# Example usage
if __name__ == "__main__":
    # Generate test signals
    fs = 1000.0  # Hz
    t = np.arange(0, 10, 1/fs)

    # Signal 1: sinusoid + noise
    x1 = np.sin(2*np.pi*10*t) + 0.5*np.random.randn(len(t))

    # Signal 2: delayed + attenuated version + independent noise
    x2 = 0.8*np.sin(2*np.pi*10*t - 0.1) + 0.3*np.random.randn(len(t))

    # Analyze
    analyzer = CoherenceAnalyzer(fs)
    f, C, F = analyzer.compute_coherence(x1, x2)

    # Verify conservation
    print(f"C + F = 1 verified: {analyzer.verify_conservation(C, F)}")

    # Find operational point
    op_point = analyzer.find_operational_point(C)
    print(f"Operational point: C = {op_point['C_actual']:.4f}, "
          f"F = {op_point['F_actual']:.4f}")

    # Mutual information lower bound
    I_LB = analyzer.mutual_information_lower_bound(f, F, fc=100.0)
    print(f"Mutual information lower bound: {I_LB:.4f} bits")

    # Plot
    fig, axes = plt.subplots(2, 1, figsize=(10, 8))

    axes[0].semilogx(f, C, label='C(f)')
    axes[0].semilogx(f, F, label='F(f)')
    axes[0].axhline(0.86, color='r', linestyle='--', label='Target C')
    axes[0].set_ylabel('Magnitude')
    axes[0].set_title('Coherence and Fluctuation Spectra')
    axes[0].legend()
    axes[0].grid(True)

    axes[1].semilogx(f, C + F)
    axes[1].axhline(1.0, color='r', linestyle='--')
    axes[1].set_xlabel('Frequency (Hz)')
    axes[1].set_ylabel('C + F')
    axes[1].set_title('Conservation Law Verification')
    axes[1].grid(True)

    plt.tight_layout()
    plt.savefig('coherence_analysis.png', dpi=150)
    print("Plot saved to coherence_analysis.png")
