import numpy as np
import time
from typing import List, Tuple
from .hsi import HSI
from .arkhe_types import HexVoxel

class ImmuneSystem:
    """
    Sistema Imunológico Digital: Patrols the HSI to detect and isolate 'Byzantine Infections'
    (nodes with high entropy or instability).
    """
    def __init__(self, hsi: HSI, sanity_threshold: float = 0.3):
        self.hsi = hsi
        self.sanity_threshold = sanity_threshold # Gamma in concept
        self.history = {} # coords -> list of recent intention amplitudes

    def patrol(self):
        """
        Linfócito de Consenso: Scans voxels for instability and divergence.
        Implements: |Φ_node - Φ_neighbors| > ε and S(t) = |dF/dt|
        """
        alerts = []
        for coords, voxel in self.hsi.voxels.items():
            # 1. Diferencial Semântico (S(t) = |dF/dt|)
            current_f = voxel.genome.f
            if coords not in self.history:
                self.history[coords] = []

            self.history[coords].append(current_f)
            if len(self.history[coords]) > 10:
                self.history[coords].pop(0)

            if len(self.history[coords]) >= 3:
                h = self.history[coords]
                dt = 1.0 # simulated timestep
                dF = (h[-1] - h[-2]) / dt
                d2F = (h[-1] - 2*h[-2] + h[-3]) / (dt*dt)

                voxel.stability_index = np.clip(1.0 - np.abs(dF) * 5, 0, 1)

                # Signature of Byzantine Infection (Imminent collapse)
                if dF < -0.3 and d2F < -1.0:
                    alerts.append((coords, "imminent_collapse"))
                elif np.abs(dF) > self.sanity_threshold:
                    alerts.append((coords, "instability"))

            # 2. Diferencial de Coerência (|Φ_node - Φ_neighbors| > ε)
            neighbors = self.hsi.get_neighbors(coords)
            valid_neighbors = [self.hsi.voxels[nb].phi for nb in neighbors if nb in self.hsi.voxels]
            if valid_neighbors:
                avg_nb_phi = np.mean(valid_neighbors)
                phi_diff = np.abs(voxel.phi - avg_nb_phi)
                if phi_diff > 0.12: # ε = 0.12 as per specification
                    alerts.append((coords, "divergence"))

        # Process Alerts
        for coords, reason in alerts:
            if not self.hsi.voxels[coords].is_quarantined:
                if reason == "imminent_collapse":
                    self.informational_tourniquet(coords)
                else:
                    # Early Warning (Cytokine Pulse)
                    self.hsi.voxels[coords].memory_bias -= 0.05 # Negative bias

    def informational_tourniquet(self, coords):
        """
        Isolates a voxel from the collective.
        """
        voxel = self.hsi.voxels[coords]
        voxel.is_quarantined = True

        # 1. Edge Pruning: Weight zeroing (Poda de Arestas)
        voxel.weights *= 0.0

        # 2. Sequestro Quântico: Force Absolute Doubt (Superposition)
        # In this implementation, we reset its intention state
        voxel.state[:6] = complex(1/math.sqrt(6), 0)
        voxel.state[6] = complex(0, 0)

        # 3. Coerência is zeroed for consensus ignore
        voxel.phi_data = 0.0
        voxel.phi_field = 0.0

        print(f"  [Immune System] Informational Tourniquet applied to {coords}. Node isolated.")

    def release_quarantine(self, coords):
        voxel = self.hsi.voxels[coords]
        voxel.is_quarantined = False
        voxel.weights = np.ones(6, dtype=np.float32)
        print(f"  [Immune System] Node {coords} released from quarantine.")
