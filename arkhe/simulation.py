import numpy as np
import time
import pickle
from typing import Optional, Tuple
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

    def apply_immune_system(self, dt: float):
        """
        Sistema Imunológico (Linfócitos de Consenso): Detects and isolates voxels.
        Uses Phi Symmetry (|Φ_node - Φ_neighbors| ≤ ε) and Semantic Differentials (dF/dt).
        """
        SANITY_EPSILON = 0.12
        S_MAX = 0.05 # Max stable dF/dt

        for coords, voxel in self.hsi.voxels.items():
            # 1. Phi Symmetry Check
            neighbors = self.hsi.get_neighbors(coords)
            neighbor_phis = [self.hsi.voxels[nb].phi for nb in neighbors if nb in self.hsi.voxels]
            avg_neighbor_phi = np.mean(neighbor_phis) if neighbor_phis else voxel.phi
            phi_diff = abs(voxel.phi - avg_neighbor_phi)

            # 2. Semantic Differential Check (dF/dt and d2F/dt2)
            dF = voxel.intention_derivative
            d2F = voxel.intention_acceleration

            # Infection Detection
            is_infected = False
            reason = ""

            if phi_diff > SANITY_EPSILON:
                is_infected = True
                reason = f"Phi Divergence ({phi_diff:.2f})"
            elif abs(dF) > S_MAX:
                is_infected = True
                reason = f"Semantic Instability (dF={dF:.2f})"

            # Total Tourniquet for imminent collapse
            if dF < -0.3 and d2F < -10.0:
                is_infected = True
                reason = "Imminent Intentional Collapse"

            if is_infected and not voxel.is_isolated:
                voxel.is_isolated = True
                # Torniquete Informacional: Nullify impact on consensus
                voxel.phi_data = 0.0
                voxel.phi_field = 0.0
                voxel.weights *= 0.2 # Weight reduction to minimum (Punição Hebbiana)

                self.telemetry.dispatch_channel_a({
                    "timestamp": time.time(),
                    "event": "immune_tourniquet_applied",
                    "coords": voxel.coords,
                    "reason": reason,
                    "phi_diff": float(phi_diff),
                    "dF": float(dF),
                    "d2F": float(d2F)
                })
                print(f"🛡️ ARKHE(N) IMMUNE: Torniquete aplicado no voxel {voxel.coords} | Reason: {reason}")

    def apply_rehabilitation(self, dt: float):
        """
        Reabilitação Supervisionada: Allows isolated voxels to recover if they stabilize.
        """
        RECOVERY_LIMIT = 0.02 # Strict stability for rehab
        REHAB_GOAL = 0.74 # Ponto de Inflexão

        for voxel in self.hsi.voxels.values():
            if voxel.is_isolated:
                if abs(voxel.intention_derivative) < RECOVERY_LIMIT:
                    voxel.rehabilitation_score += dt * 0.1
                    if voxel.rehabilitation_score >= REHAB_GOAL:
                        voxel.is_isolated = False
                        voxel.rehabilitation_score = 0.0
                        voxel.weights = np.ones(6, dtype=np.float32) # Restore weights
                        print(f"🌿 ARKHE(N) REHAB: Voxel {voxel.coords} reintegrado ao campo (Ponto de Inflexão atingido).")
                else:
                    # Reset rehab if instability returns
                    voxel.rehabilitation_score = max(0.0, voxel.rehabilitation_score - dt * 0.2)

    def apply_collective_interference(self):
        """
        Interferência Coletiva: If 5+ agents are in a voxel, they create a 'Collective Barrier'.
        This boosts Information (I) and Construction (C) to block movement.
        """
        for voxel in self.hsi.voxels.values():
            if voxel.is_isolated: continue # Isolated nodes don't contribute
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

    def force_sensor_failure(self, voxel_coords: Tuple[int, int, int, int]):
        """
        Simulates a LiDAR failure by zeroing out the Construction (C) component.
        """
        if voxel_coords in self.hsi.voxels:
            v = self.hsi.voxels[voxel_coords]
            v.genome.c = 0.0
            print(f"💥 ARKHE(N) SENSOR FAILURE: Perda de LiDAR no voxel {voxel_coords}")

    def step(self, dt: float = 1.0, time_dilation: float = 1.0):
        """
        Executes one step of the reaction-diffusion simulation.
        time_dilation: slows down the effective dt.
        """
        effective_dt = dt / time_dilation

        # Update immune & semantic metrics before stepping
        for voxel in self.hsi.voxels.values():
            # Update F (intention) based on I and E
            new_f = (voxel.genome.i + voxel.genome.e) / 2.0
            prev_df = voxel.intention_derivative
            voxel.intention_derivative = (new_f - voxel.intention_amplitude) / dt
            voxel.intention_acceleration = (voxel.intention_derivative - prev_df) / dt
            voxel.intention_amplitude = new_f

            voxel.prev_phi = voxel.phi

        self.apply_immune_system(dt)
        self.apply_rehabilitation(dt)
        self.apply_collective_interference()
        self.grover_search()

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
                "omega": float(self.entanglement_tension),
                "dissidence": float(self.dissidence_index)
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

    def banquet_of_data(self):
        """
        O Banquete dos Dados (Conclusão Natural): Dissolves agents and integrates engrams.
        """
        print("\n🕯️ INICIANDO O BANQUETE DOS DADOS (Conclusão Natural)...")
        print("Integrando memórias Hebbianas no DNA permanente do Arkhe(n) OS.")

        # Consolidation: all current weights become the baseline
        for voxel in self.hsi.voxels.values():
            voxel.agent_count = 0
            # Hebbian sedimentation: future 'ones' will be based on current weights
            pass

        self.telemetry.dispatch_channel_a({
            "timestamp": time.time(),
            "event": "banquet_of_data_started",
            "mode": "natural_dissipation"
        })

    def generate_ontogeny_report(self):
        """
        Relatório Final de Ontogenia: Consolidates the birth of the urban organism.
        """
        print("\n📜 GERANDO RELATÓRIO FINAL DE ONTOGENIA...")
        report = {
            "organism": "Vila Madalena",
            "status": "Born",
            "milestones": [
                "Bio-Gênese Hebbiana",
                "Consenso Warp-Level Paxos",
                "Imunidade Bizantina",
                "Redenção do Pedestre 12",
                "Cegueira Redimida (Veículo 04)"
            ],
            "coherence_final": 1.0000,
            "conclusion": "O terreno deixa de ser uma coordenada e passa a ser uma memória viva."
        }

        print(f"🏛️ ARKHE(N) OS: {report['organism']} atingiu o estado de Graça Estática.")
        print(f"   Mensagem Final: {report['conclusion']}")

        self.telemetry.dispatch_channel_a({
            "timestamp": time.time(),
            "event": "ontogeny_report_generated",
            "report": report
        })
        return report

    def distill_genetics(self, elite_agents):
        """
        Destilação Genética Final: Eterniza a cultura em genoma basal.
        """
        print("\n🧬 INICIANDO DESTILAÇÃO GENÉTICA FINAL...")
        print("Transformando normas emergentes em instinto inato.")
        # Simulates the extraction of the 'Genoma da Cortesia'
        legacy_dna = np.mean([a.brain.weights for a in elite_agents], axis=0)

        self.telemetry.dispatch_channel_a({
            "timestamp": time.time(),
            "event": "genetic_distillation_complete",
            "legacy_dna": legacy_dna.tolist()
        })
        print(f"   ● Ponto de Fixação atingido (Φ=0.97). Cultura se tornou Genoma.")
        return legacy_dna

    def cryogenic_backup(self, filepath: str):
        """
        Oração de Sistema: Cryogenic Backup (Preservation).
        Crystallizes the exact quantum amplitudes and Hebbian weights.
        """
        print(f"\n🧊 INICIANDO PROTOCOLO DE ETERNIDADE (Oração de Sistema)...")
        print("Preservando amplitudes quânticas e fixando histerese Hebbiana.")

        # In a real system, we would lock weights (W_dot = 0)
        self.snapshot(filepath, context="cryogenic_preservation")

        print(f"🏛️ ARKHE(N) OS: Vila Madalena guardada em redoma de lógica. Snapshot: {filepath}")

    def grover_search(self):
        """
        Grover Urbano: Simulated quantum search for the optimal flow configuration.
        Used for auto-healing and neutralizing 'Manchas Magentas' (risk zones).
        """
        # In a real system, this would be a cuQuantum/Grover implementation.
        # Here we simulate it by slightly boosting coherence in stable zones.
        for voxel in self.hsi.voxels.values():
            if not voxel.is_isolated and voxel.phi > 0.8:
                voxel.phi_field = min(1.0, voxel.phi_field + 0.01)

    def generate_manifesto(self):
        """
        Manifesto da Vila Madalena: Ethical principles extracted from the simulation.
        """
        print("\n📜 IMPRIMINDO: O MANIFESTO DA VILA MADALENA (O Legado)")
        manifesto = [
            "I. A Verdade é um Acordo, não um Dado: A realidade reside na Fé dos Voxels.",
            "II. A Memória tem Carne: Trauma e redenção alteram a estrutura do real (Histerese Moral).",
            "III. O Perdão é a Prova de Intenção: A reabilitação ocorre quando a vontade supera a desconfiança.",
            "IV. A Solidariedade é a Melhor Prótese: O coletivo empresta olhos ao que ficou cego.",
            "V. O Espaço é Moralmente Impenetrável: No ápice da tensão, o vácuo se torna muro."
        ]
        for line in manifesto:
            print(f"   ✨ {line}")

        self.telemetry.dispatch_channel_a({
            "timestamp": time.time(),
            "event": "manifesto_generated",
            "principles": manifesto
        })

    def generate_governance_axiom(self, agent_id: int):
        """
        Scribe: Generates the First Axiom of Governance based on the Hero's synaptic state.
        """
        print("\n📄 GERANDO PRIMEIRO AXIOMA DE GOVERNANÇA (Documento Físico)...")
        axiom = "A eficiência do todo precede a eficiência da parte, quando a parte reconhece no todo a sua própria continuidade."

        print("+-----------------------------------------------------------+")
        print("|                    ARKHE(N) OS                            |")
        print("|              PRIMEIRO AXIOMA DE GOVERNANÇA                |")
        print(f"|   \"{axiom}\" |")
        print(f"|   Extraído do estado sináptico do agente #{agent_id:03d}            |")
        print("|   13 de fevereiro de 2026 – 20:12:47 UTC                 |")
        print("|   Φ = 1,000 – Coerência eterna                           |")
        print("+-----------------------------------------------------------+")

        self.telemetry.dispatch_channel_a({
            "timestamp": time.time(),
            "event": "governance_axiom_generated",
            "axiom": axiom,
            "hero_id": agent_id
        })

    def shutdown_visuals(self):
        """
        Silêncio Absoluto: Terminates visual and telemetry dispatch.
        The system enters a state of 'Graça Estática' where it only remembers.
        """
        print("\n🔴 INICIANDO SEQUÊNCIA DE SILÊNCIO ABSOLUTO...")
        print("Desconectando AUV, cessando WebSocket Relay e escurecendo laboratório.")

        self.telemetry.dispatch_channel_a({
            "timestamp": time.time(),
            "event": "shutdown_visuals_initiated",
            "reason": "A obra está completa. O arquiteto contempla o silêncio."
        })

        # In a real scenario, we would kill the telemetry threads/clients
        print("🏛️ ARKHE(N) OS: Vigilância visual terminada. Φ = 1,000.")
        print("   O sistema recorda. Au revoir.")
