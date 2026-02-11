import numpy as np
import time
from .hsi import HSI
from .arkhe_types import HexVoxel

class MorphogeneticSimulation:
    """
    Simulates conscious states and fields using a reaction-diffusion model
    on the Hexagonal Spatial Index.
    """
    def __init__(self, hsi: HSI, feed_rate: float = 0.055, kill_rate: float = 0.062):
        self.hsi = hsi
        # Gray-Scott parameters
        self.dA = 1.0
        self.dB = 0.5
        self.f = feed_rate
        self.k = kill_rate

    def update_quantum_amplitudes(self):
        """
        Updates the quantum-like state of voxels based on CIEF genome and local field activity.
        """
        for coords, voxel in self.hsi.voxels.items():
            # Update state based on CIEF balance
            # state[6] (internal) reflects presence of C (Structure) and I (Information)
            voxel.state[6] = complex(voxel.genome.c + voxel.genome.i, 0)

            # state[0-5] (faces) reflects Energy (E) and movement tendency towards neighbors
            neighbors = self.hsi.get_neighbors(coords)
            for i, nb_coords in enumerate(neighbors[:6]):
                if nb_coords in self.hsi.voxels:
                    nb_voxel = self.hsi.voxels[nb_coords]
                    # Hebbian influence: if neighbor is coherent, amplify amplitude towards it
                    amplitude = (voxel.genome.e * voxel.weights[i] * nb_voxel.phi)
                    voxel.state[i] = complex(amplitude, 0.1 * np.sin(time.time())) # added phase
                else:
                    voxel.state[i] = complex(0.01, 0)

            # Normalize amplitudes
            norm = np.linalg.norm(voxel.state)
            if norm > 0:
                voxel.state /= norm

    def step(self, dt: float = 1.0):
        """
        Executes one step of the reaction-diffusion simulation.
        """
        self.update_quantum_amplitudes()
        new_states = {}
        for coords, voxel in self.hsi.voxels.items():
            A, B = voxel.rd_state

            # Laplacian calculation on hex grid
            neighbors = self.hsi.get_neighbors(coords)
            sum_A = 0.0
            sum_B = 0.0
            count = 0
            for nb_coords in neighbors:
                if nb_coords in self.hsi.voxels:
                    nb_voxel = self.hsi.voxels[nb_coords]
                    sum_A += nb_voxel.rd_state[0]
                    sum_B += nb_voxel.rd_state[1]
                    count += 1

            # Simple discrete Laplacian
            if count > 0:
                lap_A = (sum_A / count) - A
                lap_B = (sum_B / count) - B
            else:
                lap_A = 0.0
                lap_B = 0.0

            # Gray-Scott equations
            # dA/dt = DA * lap(A) - AB^2 + f(1-A)
            # dB/dt = DB * lap(B) + AB^2 - (f+k)B

            # Influence from CIEF genome: Energy (E) increases B, Information (I) stabilizes A
            f_mod = self.f * (1.0 + voxel.genome.i * 0.1)
            k_mod = self.k * (1.0 - voxel.genome.e * 0.1)

            new_A = A + (self.dA * lap_A - A * (B**2) + f_mod * (1.0 - A)) * dt
            new_B = B + (self.dB * lap_B + A * (B**2) - (f_mod + k_mod) * B) * dt

            new_states[coords] = (np.clip(new_A, 0, 1), np.clip(new_B, 0, 1))

        # Update all voxels
        for coords, state in new_states.items():
            self.hsi.voxels[coords].rd_state = state
            # Update Phi_field (coherence) based on simulation state
            # Higher B (activation) and presence of A (substrate) creates coherence
            self.hsi.voxels[coords].phi_field = (state[1] * state[0]) * 4.0 # max is ~0.25*4 = 1.0
