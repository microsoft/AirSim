import numpy as np
from typing import Dict, Tuple, List
from .hsi import HSI

class HSISnapshot:
    """
    Stores a snapshot of HSI weights for memory analysis.
    """
    def __init__(self, hsi: HSI):
        self.weights: Dict[Tuple[int, int, int, int], np.ndarray] = {
            coords: voxel.weights.copy() for coords, voxel in hsi.voxels.items()
        }

class MemoryAnalyzer:
    """
    Analyzes the 'engram' or 'scars' of learning in the HSI.
    """
    def __init__(self, hsi: HSI):
        self.hsi = hsi

    def calculate_delta(self, baseline: HSISnapshot) -> Dict[Tuple[int, int, int, int], np.ndarray]:
        """
        Calculates the change in weights compared to a baseline.
        """
        deltas = {}
        for coords, voxel in self.hsi.voxels.items():
            if coords in baseline.weights:
                delta = voxel.weights - baseline.weights[coords]
            else:
                # If new voxel, assume initial weights were 1.0 (as per HexVoxel init)
                delta = voxel.weights - np.ones(6, dtype=np.float32)

            if np.any(np.abs(delta) > 1e-5):
                deltas[coords] = delta
        return deltas

    def generate_engram_report(self, baseline: HSISnapshot):
        """
        Identifies preferential flow paths and reinforced areas.
        """
        deltas = self.calculate_delta(baseline)

        print("\n🧠 ARKHE(N) MEMORY REPORT: HEBBIAN ENGRAM ANALYSIS")
        print("-" * 50)

        # Summary
        total_reinforced = len(deltas)
        if total_reinforced == 0:
            print("No significant learning detected in the field.")
            return

        max_delta = 0.0
        strongest_voxel = None

        for coords, d in deltas.items():
            m = np.max(d)
            if m > max_delta:
                max_delta = m
                strongest_voxel = coords

        print(f"Total Reinforced Voxels: {total_reinforced}")
        print(f"Strongest Sinaptic Reinforcement: {max_delta:.4f} at {strongest_voxel}")

        # Identify "Vias Preferenciais" (Preferential Paths)
        print("\nPreferential Flow Paths (Strongest connections):")
        sorted_deltas = sorted(deltas.items(), key=lambda x: np.max(x[1]), reverse=True)
        for coords, d in sorted_deltas[:5]:
            direction = np.argmax(d)
            print(f"  Voxel {coords} -> Direction {direction} (Delta: {d[direction]:.4f})")

        print("-" * 50)
        return deltas
