import numpy as np
import time
from typing import List, Tuple
from .hsi import HSI
from .arkhe_types import HexVoxel
from .immune import ImmuneSystem

class MorphogeneticSimulation:
    """
    Simulates conscious states and fields using a reaction-diffusion model
    on the Hexagonal Spatial Index.
    """
    def __init__(self, hsi: HSI, feed_rate: float = 0.055, kill_rate: float = 0.062, learning_rate: float = 0.01):
        self.hsi = hsi
        self.immune = ImmuneSystem(hsi)
        # Gray-Scott parameters
        self.dA = 1.0
        self.dB = 0.5
        self.f = feed_rate
        self.k = kill_rate
        # Hebbian Learning Rate
        self.eta = learning_rate

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

            # Interference/Conflict detection:
            # If opposite faces have high amplitudes, it indicates conflict/frustration
            conflict = 0.0
            for i in range(3): # Check pairs 0-3, 1-4, 2-5 (opposite sides of hexagon)
                amp_i = np.abs(voxel.state[i])
                amp_opp = np.abs(voxel.state[i+3])
                # Interference is high if both are trying to push in opposite directions
                conflict += amp_i * amp_opp
            voxel.conflict_level = np.clip(conflict * 2.0, 0, 1)

    def update_hebbian_weights(self):
        """
        Hebbian Learning Rule: Reinforces connections between voxels that are
        simultaneously coherent (Φ).
        Delta_w = eta * (phi_i * phi_j)
        """
        for coords, voxel in self.hsi.voxels.items():
            neighbors = self.hsi.get_neighbors(coords)
            for i, nb_coords in enumerate(neighbors[:6]):
                if nb_coords in self.hsi.voxels:
                    nb_voxel = self.hsi.voxels[nb_coords]
                    # LTP (Long-Term Potentiation)
                    delta_w = self.eta * (voxel.phi * nb_voxel.phi)
                    voxel.weights[i] = np.clip(voxel.weights[i] + delta_w, 0, 2.0)

    def relax(self, dt: float = 1.0, b_reduction: float = 0.03):
        """
        Simulates post-event homeostasis (Relaxation phase).
        Reduces activity and lets the field return to equilibrium.
        """
        new_states = {}
        for coords, voxel in self.hsi.voxels.items():
            A, B = voxel.rd_state

            # Reduce activity (B) to simulate relaxation
            B *= (1.0 - b_reduction)

            # Laplacian calculation
            neighbors = self.hsi.get_neighbors(coords)
            sum_A = sum(self.hsi.voxels[nb].rd_state[0] for nb in neighbors if nb in self.hsi.voxels)
            sum_B = sum(self.hsi.voxels[nb].rd_state[1] for nb in neighbors if nb in self.hsi.voxels)
            count = sum(1 for nb in neighbors if nb in self.hsi.voxels)

            if count > 0:
                lap_A = (sum_A / count) - A
                lap_B = (sum_B / count) - B
            else:
                lap_A = lap_B = 0.0

            # Simplified dynamics for relaxation
            new_A = A + (self.dA * lap_A) * dt
            new_B = B + (self.dB * lap_B - self.k * B) * dt

            new_states[coords] = (np.clip(new_A, 0, 1), np.clip(new_B, 0, 1))

        for coords, state in new_states.items():
            self.hsi.voxels[coords].rd_state = state
            # Update coherence (phi_field) - should decrease during relaxation
            self.hsi.voxels[coords].phi_field *= 0.9

    def detect_collective_entanglement(self, threshold: int = 5):
        """
        Detects groups of agents with similar intentions and creates a probability barrier.
        Implements resilience: if a group was already entangled, it stays coherent even
        if membership drops slightly below threshold (Hysteresis).
        """
        intention_clusters = {} # direction -> list of coords
        for coords, voxel in self.hsi.voxels.items():
            if np.abs(voxel.state[6]) < 0.5: # Likely a pedestrian
                max_dir = np.argmax(np.abs(voxel.state[:6]))
                if max_dir not in intention_clusters:
                    intention_clusters[max_dir] = []
                intention_clusters[max_dir].append(coords)

        for direction, members in intention_clusters.items():
            # Check for current barrier state in these voxels
            already_entangled = any(self.hsi.voxels[c].phi_field == 1.0 for c in members)

            # Resilience threshold is lower than initial threshold (Hysteresis)
            resilience_threshold = threshold - 1 if already_entangled else threshold

            if len(members) >= resilience_threshold:
                # Emergence of Order / Maintenance of Order
                for coords in members:
                    voxel = self.hsi.voxels[coords]
                    # Maintenance of "Muro de Probabilidade"
                    voxel.memory_bias = np.clip(voxel.memory_bias + 0.1, 0, 1.0)
                    voxel.rd_state = (0.0, 1.0)
                    voxel.phi_field = 1.0
            else:
                # Dissolution of barrier
                for coords in members:
                    self.hsi.voxels[coords].phi_field *= 0.5

    def calculate_entanglement_tension(self, leader_coords, target_vehicle_coords):
        """
        Calculates the Entanglement Tension (tau) between two voxels.
        tau = sum | psi_a * psi_b | (Simplified tensor product magnitude)
        """
        if leader_coords in self.hsi.voxels and target_vehicle_coords in self.hsi.voxels:
            psi_leader = self.hsi.voxels[leader_coords].state
            psi_vehicle = self.hsi.voxels[target_vehicle_coords].state
            # Magnitude of the outer product
            tau = np.linalg.norm(np.outer(psi_leader, psi_vehicle))
            return tau
        return 0.0

    def betrayal_protocol(self, coords, weight_penalty: float = -0.3):
        """
        Protocolo de Judas: Sudden collapse of intention for a specific voxel.
        """
        if coords in self.hsi.voxels:
            voxel = self.hsi.voxels[coords]
            # Collapse intention amplitude
            voxel.state[:6] *= 0.1
            voxel.state[6] = complex(1.0, 0) # Return to ego/static
            # Weaken connections with group
            voxel.weights *= (1.0 + weight_penalty)
            voxel.phi_field *= 0.2
            # Set a memory bias to represent the scar
            voxel.memory_bias = -0.1
            print(f"  [Betrayal] Voxel {coords} has triggered the Judas Protocol.")

    def reconciliation_phase(self, traitor_coords, recovery_rate: float = 0.05):
        """
        Gradual trust recovery for a traitor voxel (Reconciliação).
        Implements 'Ponto de Inflexão' (0.74) and 'Aniquilação de Cicatriz'.
        """
        if traitor_coords in self.hsi.voxels:
            voxel = self.hsi.voxels[traitor_coords]

            # Trust recovery
            voxel.rehabilitation_index = np.clip(voxel.rehabilitation_index + recovery_rate, 0, 1.0)

            # Gradually restore weights
            target_weights = np.ones(6, dtype=np.float32)
            voxel.weights += (target_weights - voxel.weights) * (recovery_rate * 2)

            # Ponto de Inflexão (0.74)
            if voxel.rehabilitation_index >= 0.74 and voxel.memory_bias != 0:
                print(f"  ✨ [MIRACLE] Aniquilação de Cicatriz at {traitor_coords}. Memory Bias reset.")
                voxel.memory_bias = 0.0
                voxel.phi_field = 1.0 # Pure crystalline state

            if voxel.rehabilitation_index > 0.9:
                print(f"  [Reconciliation] Voxel {traitor_coords} has been fully reintegrated into the collective.")

    def materialize_memory_to_bias(self, target_coords_list: List[Tuple[int, int, int, int]]):
        """
        Frente B: Converts Hebbian weights and object context to bias voltages for physical metasurfaces.
        """
        bias_report = {}
        for coords in target_coords_list:
            if coords in self.hsi.voxels:
                voxel = self.hsi.voxels[coords]
                # V_bias = weight * scaling (e.g., 250mV max)
                avg_weight = np.mean(voxel.weights)

                # Context-aware modulation
                context_multiplier = 1.0
                if voxel.object_label == "Pedestre 12":
                    context_multiplier = 1.5 # Sovereign agent priority
                elif voxel.object_label == "Thermal Hazard":
                    context_multiplier = 2.0 # Critical response

                bias_mv = avg_weight * 250.0 * context_multiplier
                voxel.memory_bias = avg_weight * 0.1 * context_multiplier # Feedback to field
                bias_report[coords] = bias_mv

        if bias_report:
            print(f"  [Materialization] Physical Metasurface Updated. Max Bias: {max(bias_report.values()):.2f} mV")
        return bias_report

    def simulate_radiative_cooling_vortex(self):
        """
        Simulates the 'Suor Radiativo' - a vortex of heat annihilation following vehicles.
        """
        for coords, voxel in self.hsi.voxels.items():
            if voxel.genome.c > 0.8: # Vehicle signature
                # Annihilate thermal energy in neighbors
                neighbors = self.hsi.get_neighbors(coords)
                for nb in neighbors:
                    if nb in self.hsi.voxels:
                        # Annihilation vortex
                        self.hsi.voxels[nb].genome.e *= 0.8
                        self.hsi.voxels[nb].rd_state = (self.hsi.voxels[nb].rd_state[0], self.hsi.voxels[nb].rd_state[1] * 0.5)

    def step(self, dt: float = 1.0, time_dilation: float = 1.0):
        """
        Executes one step of the reaction-diffusion simulation with time dilation.
        """
        # Adjust dt for slow-motion observation
        effective_dt = dt / time_dilation

        # Immune Patrol
        self.immune.patrol()

        self.update_quantum_amplitudes()
        self.update_hebbian_weights()
        self.detect_collective_entanglement()
        self.simulate_radiative_cooling_vortex()
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

            # Influence from Object Context
            if voxel.object_label == "Vehicle":
                f_mod *= 1.2
            elif voxel.object_label == "Pedestrian" or voxel.object_label == "Pedestre 12":
                f_mod *= 0.8 # Slower, more stable diffusion

            new_A = A + (self.dA * lap_A - A * (B**2) + f_mod * (1.0 - A) + voxel.memory_bias) * effective_dt
            new_B = B + (self.dB * lap_B + A * (B**2) - (f_mod + k_mod) * B) * effective_dt

            new_states[coords] = (np.clip(new_A, 0, 1), np.clip(new_B, 0, 1))

        # Update all voxels
        for coords, state in new_states.items():
            self.hsi.voxels[coords].rd_state = state
            # Update Phi_field (coherence) based on simulation state
            # Higher B (activation) and presence of A (substrate) creates coherence
            self.hsi.voxels[coords].phi_field = (state[1] * state[0]) * 4.0 # max is ~0.25*4 = 1.0
