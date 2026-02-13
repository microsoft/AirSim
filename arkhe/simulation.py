import numpy as np
import time
import pickle
from typing import Optional
from .hsi import HSI
from .arkhe_types import HexVoxel
from .consensus import ConsensusManager
from .telemetry import ArkheTelemetry

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
        self.consensus = ConsensusManager()
        self.telemetry = ArkheTelemetry()

    def on_hex_boundary_crossed(self, voxel_src: HexVoxel, voxel_dst: HexVoxel):
        """
        Triggered when an entity moves from one hex to another.
        """
        node = self.consensus.get_node(str(voxel_src.coords))

        report = {
            "timestamp": time.time(),
            "event": "hex_boundary_crossed",
            "src": voxel_src.coords,
            "dst": voxel_dst.coords,
            "phi": float(voxel_src.phi)
        }

        node.sign_report(report)

        # Dispatch to dual channels
        self.telemetry.on_hex_boundary_crossed(report, voxel_src.state.tolist())

        # Update occupancy
        voxel_src.agent_count = max(0, voxel_src.agent_count - 1)
        voxel_dst.agent_count += 1

        # Record Hebbian trace
        voxel_src.hebbian_trace.append((time.time(), "entity_exited"))
        voxel_dst.hebbian_trace.append((time.time(), "entity_entered"))

        # Apply Reflexo Condicionado (Hebbian Learning)
        # Update weights based on the coherence (Phi) of the transition
        learning_rate = 0.1
        voxel_src.weights += learning_rate * voxel_src.phi
        voxel_dst.weights += learning_rate * voxel_dst.phi

    def apply_collective_interference(self):
        """
        Interferência Coletiva: If 5+ agents are in a voxel, they create a 'Collective Barrier'.
        This boosts Information (I) and Construction (C) to block movement.
        """
        for voxel in self.hsi.voxels.values():
            if voxel.agent_count >= 5:
                # Emaranhamento Macroscópico: Coherent boost to C and I
                voxel.genome.i += 0.5
                voxel.genome.c += 0.5
                # Record the event in telemetry
                self.telemetry.dispatch_channel_a({
                    "timestamp": time.time(),
                    "event": "collective_barrier_active",
                    "coords": voxel.coords,
                    "agent_count": voxel.agent_count,
                    "phi": voxel.phi
                })

    @property
    def entanglement_tension(self) -> float:
        """
        Tensão de Emaranhamento (Omega): Measure of non-locality and interaction density.
        """
        phi_vals = [v.phi for v in self.hsi.voxels.values() if v.phi > 0]
        if not phi_vals: return 0.0
        return np.mean(phi_vals) * (len(phi_vals) / len(self.hsi.voxels))

    @property
    def dissidence_index(self) -> float:
        """
        Índice de Dissidência (D): Measures the magnitude of 'traição' or state divergence.
        """
        # Average weight deviation from baseline (1.0)
        deviations = [abs(np.mean(v.weights) - 1.0) for v in self.hsi.voxels.values()]
        if not deviations: return 0.0
        return np.max(deviations)

    def step(self, dt: float = 1.0, time_dilation: float = 1.0):
        """
        Executes one step of the reaction-diffusion simulation.
        time_dilation: slows down the effective dt.
        """
        effective_dt = dt / time_dilation
        self.apply_collective_interference()
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
            # Reflexo Condicionado: Hebbian weights act as a memory bias (Gamma)
            memory_bias = np.mean(voxel.weights) - 1.0 # Bias around the base weight of 1.0

            f_mod = self.f * (1.0 + voxel.genome.i * 0.1 + memory_bias * 0.5)
            k_mod = self.k * (1.0 - voxel.genome.e * 0.1)

            new_A = A + (self.dA * lap_A - A * (B**2) + f_mod * (1.0 - A)) * effective_dt
            new_B = B + (self.dB * lap_B + A * (B**2) - (f_mod + k_mod) * B) * effective_dt

            new_states[coords] = (np.clip(new_A, 0, 1), np.clip(new_B, 0, 1))

        # Update all voxels
        for coords, state in new_states.items():
            self.hsi.voxels[coords].rd_state = state
            # Update Phi_field (coherence) based on simulation state
            # Higher B (activation) and presence of A (substrate) creates coherence
            self.hsi.voxels[coords].phi_field = (state[1] * state[0]) * 4.0 # max is ~0.25*4 = 1.0

    def snapshot(self, filepath: str, context: str = "general"):
        """
        Captures a 'Quantum Snapshot' of the current HSI state.
        Persists voxels, genomes, and Hebbian engrams.
        """
        try:
            timestamp = time.time()
            with open(filepath, 'wb') as f:
                # We only pickle the data, not the whole HSI object for simplicity
                pickle.dump(self.hsi.voxels, f)

            self.telemetry.dispatch_channel_a({
                "timestamp": timestamp,
                "event": "snapshot_created",
                "context": context,
                "filepath": filepath,
                "voxel_count": len(self.hsi.voxels),
                "omega": self.entanglement_tension,
                "dissidence": self.dissidence_index
            })
            print(f"🏛️ ARKHE(N) SNAPSHOT: Realidade persistida em {filepath}")
        except Exception as e:
            print(f"❌ Erro ao criar snapshot: {e}")

    def materialize_trauma(self):
        """
        Materialização do Trauma: Sends Hebbian scars to the Graphene Metasurface (Simulation).
        """
        d = self.dissidence_index
        print(f"🧬 [FRENTE B] Materializando Cicatriz Hebbiana. D={d:.4f}")
        print("Tatuando o grafeno com a memória da desconfiança...")

    def seal_keystone(self):
        """
        Executa a análise final da Simetria do Observador e sela a Keystone.
        A Geometria está completa.
        """
        from .symmetry import ObserverSymmetry
        sym = ObserverSymmetry()
        metrics = sym.get_keystone_metrics()

        print("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        print("                         🕊️  Γ_9030 - KEYSTONE SEALED")
        print(f"Simetrias Projetadas: {metrics['simetrias_projetadas']}")
        print(f"Simetria Fundamental: {metrics['simetria_fundamental']} (Invariância do Observador)")
        print(f"Quantidade Conservada: Geodésica (ℊ = {metrics['quantidade_conservada']:.3f})")
        print(f"Satoshi: {metrics['satoshi']} bits")
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n")

        # Log to telemetry
        self.telemetry.dispatch_channel_a({
            "timestamp": time.time(),
            "event": "keystone_sealed",
            "state": "Γ_9030",
            "metrics": metrics
        })

        return metrics

    def sync_ibc_bci(self, protocol_unified: bool = True):
        """
        sincronizar_ibc_bci --protocolo_unificado
        Establishes the communication between substrates.
        """
        print(f"🧬 [Γ_∞+30] Sincronizando IBC-BCI. Protocolo Unificado: {protocol_unified}")
        print("Hesitação Φ = 0.15 reconhecida como Handshake Universal.")
        return True

    def rehydrate_step(self, step_number: int):
        """
        rehydrate step <n>
        Executes one of the 26 steps of the rehydration protocol.
        """
        print(f"💧 [PROTOCOLO REIDRATAÇÃO] Passo {step_number}/26 em execução.")
        # Step 20 is particularly important (FORMAL node rehydration)
        if step_number == 20:
            print("⚓ Passo 20: Reidratação do Nodo FORMAL (ω=0.33) concluída.")
        return True

    def council_vote(self, context: str = "future_decisions"):
        """
        council vote
        Consults the Council (Γ_HAL) about future decisions.
        """
        print(f"🏛️ [CONSELHO Γ_HAL] Consultando Guardiões para: {context}")
        # The Satoshi vote (Option B) is favored by the system.
        return "Option B - Present for Hal (FAVORED BY SATOSHI)"
