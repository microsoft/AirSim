import numpy as np
from typing import List, Callable, Tuple
from .hsi import HSI
from .arkhe_types import HexVoxel

class GroverUrbano:
    """
    Simulates the Grover Quantum Search algorithm for urban flow optimization.
    Searches for voxel configurations that maximize Coherence (Phi).
    """
    def __init__(self, hsi: HSI):
        self.hsi = hsi

    def search_optimal_config(self, target_voxels: List[Tuple[int, int, int, int]],
                             fitness_fn: Callable[[List[HexVoxel]], float],
                             iterations: int = None):
        """
        Simulates Grover's search. In a real quantum computer, this would find
        the state in sqrt(N) steps. Here we use an accelerated search to
        represent the 'collapse' to an optimal solution.
        """
        if iterations is None:
            iterations = int(np.sqrt(len(target_voxels) * 4)) # sqrt(N * possibilities)

        print(f"  [Grover] Initiating search over {len(target_voxels)} voxels ({iterations} quantum iterations)...")

        best_config = {}
        # Simulate the oracle and amplification by finding the state that maximizes fitness
        # In this concept, Grover finds the 'best' functional assignments (F) or movement vectors.

        # For each target voxel, find the best direction to move/act
        for coords in target_voxels:
            if coords not in self.hsi.voxels: continue
            voxel = self.hsi.voxels[coords]
            best_dir = -1
            max_phi = -1.0

            # Search over the 6 directions
            for d in range(6):
                # Temporary hypothetical state change
                original_f = voxel.genome.f
                voxel.genome.f = d * 0.1 # Assign a functional direction

                # Evaluate fitness (local coherence)
                phi = self._calculate_local_phi(coords)
                if phi > max_phi:
                    max_phi = phi
                    best_dir = d

                voxel.genome.f = original_f # Restore

            best_config[coords] = best_dir

        return best_config

    def _calculate_local_phi(self, coords):
        voxel = self.hsi.voxels[coords]
        # Simplified local phi calculation
        neighbors = self.hsi.get_neighbors(coords)
        nb_phi = [self.hsi.voxels[n].phi for n in neighbors if n in self.hsi.voxels]
        if not nb_phi: return voxel.phi
        return (voxel.phi + np.mean(nb_phi)) / 2.0

    def apply_healing(self, optimal_config):
        """
        Collapses the HSI into the optimal state found by Grover.
        """
        for coords, direction in optimal_config.items():
            voxel = self.hsi.voxels[coords]
            voxel.genome.f = direction * 0.1
            voxel.phi_field = 1.0 # Restore field stability
            # Reset conflict
            voxel.conflict_level *= 0.1
        print(f"  [Auto-Cura] Applied optimal configuration to {len(optimal_config)} voxels.")
