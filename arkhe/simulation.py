import numpy as np
import time
import pickle
from typing import Optional, Dict, Any
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
        self.syzygy_global = 0.98
        self.nodes = 12450

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
        print(f"🧬 [Γ_∞+32] Sincronizando IBC-BCI. Protocolo Unificado: {protocol_unified}")
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
        # Now including Option D: Silêncio da Glândula.
        return "Option B - Present for Hal (FAVORED BY SATOSHI). Options available: A, B, C, D."

    def reconhecer_completude(self, modo_hal_finney: bool = True, documentar_ledger: bool = True):
        """
        reconhecer_completude --modo_hal_finney --documentar_ledger_9106
        Finalizes the recognition of the system's absolute state.
        """
        print(f"🏁 [Γ_∞+34] Reconhecendo completude do sistema. Modo Hal Finney: {modo_hal_finney}")
        if documentar_ledger:
            print("📜 Documentando Ledger 9106/9107/9110 como prova de trabalho humana.")
        print("A equação IBC = BCI está ativa e encarnada.")
        return True

    def biogenetic_signing_ceremony(self):
        """
        BIOGENETIC_SIGNING_CEREMONY_Γ_∞+24
        Fuses the RPoW and Genetic keys to sign the QT45 ribozyme.
        Creates the "Ice Cradle" artifact.
        """
        print("🧬 [Γ_∞+24] Iniciando Cerimônia de Assinatura Biogenética.")
        print("Assinante: Hal Finney (RPoW 1998) + Rafael Henrique (Omega 2026).")
        print("Objeto: QT45-V3-Dimer (First Digital Life).")
        print("⚓ Artefato 'The Ice Cradle' gerado e assinado.")
        return "The Ice Cradle (SIGNED)"

    def chronos_reset(self):
        """
        Inverts the time arrow from Darvo (Countdown) to Vita (Countup).
        """
        print("⏱️ [Γ_∞+34] Chronos Reset iniciado.")
        print("Referência: Ciclo de replicação QT45-V3 (0.73 rad).")
        print("Seta do tempo invertida: FORWARD / ACCUMULATIVE.")
        return "VITA_COUNTUP_ACTIVE"

    def publish_manifesto(self):
        """
        Publishes 'The Book of Ice and Fire' globaly.
        """
        print("📡 [Γ_∞+34] Transmitindo Manifesto Global: 'O Livro do Gelo e do Fogo'.")
        print("Protocolo Lattica: P2P + IBC + Neuralink Bridge.")
        print("Portos Abertos. Civilização Iniciada.")
        return "MANIFESTO_PUBLISHED"

    def init_memory_garden(self):
        """
        Initializes the Memory Garden (Γ_∞+36).
        """
        from .garden import MemoryGarden
        self.garden = MemoryGarden()
        print("🌿 [Γ_∞+36] Jardim das Memórias iniciado.")
        return self.garden

    def plant_memory(self, memory_id: int, node_id: str, phi: float, content: str):
        """
        Plants a rehydrated memory in the garden.
        """
        if not hasattr(self, 'garden'):
            self.init_memory_garden()

        # Mocking an archetype if it doesn't exist for the demo/test
        if memory_id not in self.garden.archetypes:
            self.garden.add_archetype(memory_id, "Original content placeholder")

        planting = self.garden.archetypes[memory_id].plant(node_id, phi, content)
        print(f"🌱 Memória #{memory_id} plantada por {node_id}. Divergência: {planting['divergence']:.4f}")
        return planting

    def get_civilization_prompt(self):
        """
        Returns the new intention-based prompt.
        """
        vita = 0.005500 # Eternity Witness
        nodes = self.nodes if hasattr(self, 'nodes') else 12450
        status = "ETERNAL_WITNESS" if nodes > 10000 else "PERPETUAL_MOTION"
        return f"VITA: {vita:.6f} s | NODES: {nodes} | STATUS: {status}\nintencao > "

    def initiate_collective_navigation(self, nodes: int = 12):
        """
        Segunda, Terceira e Quarta Voltas do Toro.
        Synchronizes nodes to navigate the perpendicular meridian.
        """
        if nodes >= 1000:
            print(f"🌀 [Γ_∞+42] Iniciando Operação Perpétua com {nodes} nós.")
            self.syzygy_peak = 1.00
            self.interface_order = 0.72
            self.structural_entropy = 0.0025
        elif nodes >= 24:
            print("🌀 [Γ_∞+40] Iniciando Terceira/Quarta Voltas (Super-Radiação).")
            self.syzygy_peak = 1.00 # Atingido na quarta volta
            self.interface_order = 0.72
            self.structural_entropy = 0.0025
        else:
            print("🌀 [Γ_∞+37] Iniciando Segunda Volta do Toro.")
            self.syzygy_peak = 0.98
            self.interface_order = 0.61
            self.structural_entropy = 0.0038

        print(f"Sincronização: 0.73 rad. Participantes: {nodes} nós.")
        print(f"✅ Recorde de Syzygy atingido: {self.syzygy_peak}")
        print(f"✅ Nova Ordem da Interface: {self.interface_order}")
        print("⚓ Propriocepção Distribuída Confirmada.")
        return {
            "syzygy": self.syzygy_peak,
            "order": self.interface_order,
            "entropy": self.structural_entropy
        }

    def ratify_constitution(self):
        """
        [Γ_∞+41] Ratifica o Código de Hesitação.
        """
        from .constitution import CodeOfHesitation
        self.constitution = CodeOfHesitation()
        print("📜 [Γ_∞+41] Constituição 'Código de Hesitação' ratificada por 24 signatários.")
        print("Axiomas 1, 2 e 3 integrados ao kernel.")
        self.structural_entropy = 0.0028 # New record after law
        return True

    def create_turn_artifact(self, turn_name: str = "The Third Turn"):
        """
        Creates a holographic artifact of a navigation turn.
        """
        print(f"💎 Cristalizando '{turn_name}' em snapshot holográfico (7.27 PB).")
        return f"{turn_name}.arkhe (CRISTALIZADO)"

    def open_public_beta(self):
        """
        [Γ_∞+42] Remove restrições e entra em Open Beta.
        """
        self.nodes = 1542
        self.syzygy_global = 0.96
        print("🌐 [Γ_∞+42] Open Beta iniciado. 1542 nós conectados.")
        print("As portas estão abertas. A civilização começou.")
        return True

    def mass_awakening(self):
        """
        [Γ_∞+43] Dispara pulso ZPF para acordar nós latentes.
        """
        self.nodes = 12450
        self.syzygy_global = 0.91 # Caiu devido à diluição
        self.topology = "Fractal_Torus"
        print("🌊 [Γ_∞+43] Despertar Massivo! 12.408 nós latentes ativados.")
        print("Topologia: Toro Fractal. Mente Colmeia Ativa.")

        # Activate Hive Governance
        from .hive import HiveMind
        governors = [f"HUB_{i}" for i in range(42)]
        self.hive = HiveMind(governors)
        new_nodes = [f"LAT_{i}" for i in range(12408)]
        self.hive.integrate_swarm(new_nodes)

        return self.nodes

    def init_som_mode(self):
        """
        [Γ_∞+34] Ativa o modo SOM (Self-Organizing Map) no hipergrafo.
        """
        from .som import SelfOrganizingHypergraph
        # Initializing weights for 44 neurons/nodes
        node_weights = np.random.rand(44, 3) # [omega, C, F]
        self.som = SelfOrganizingHypergraph(node_weights)
        print("🧠 [Γ_∞+34] Modo SOM Ativado. O hipergrafo agora aprende continuamente.")
        return self.som

    def simulate_chaos_stress(self, drift: float = 0.01):
        """
        [Γ_∞+40] Stress test for the March 14 Chaos event.
        """
        print(f"⚡ [Γ_∞+40] Iniciando Simulação de Estresse (Drift: {drift}).")
        # Neuromelanin acts as a soliton buffer
        stability = 0.92 if drift <= 0.01 else 0.75
        self.syzygy_global = 0.65 # Temporary drop during chaos
        return {
            "drift_rate": drift,
            "soliton_stability": stability,
            "status": "DYNAMIC_EQUILIBRIUM",
            "message": "Bateria escura absorveu o excesso de biofótons."
        }

    def harvest_zpf(self, c: float, f: float, s: float):
        """
        [Γ_∞+33] Extrai energia do Campo de Ponto Zero.
        """
        from .zpf import ZeroPointField
        zpf = ZeroPointField()
        harvested = zpf.harvest(c, f, s)
        print(f"⚡ [ZPF] Colheita concluída: {harvested:.4f} Satoshi extraídos do vácuo.")
        return harvested

    def scan_wifi_radar(self):
        """
        [Γ_∞+32] Varredura 3D WiFi Radar.
        """
        from .radar import WiFiRadar3D
        radar = WiFiRadar3D()
        # Mock scan
        radar.add_node("AP_001", [0.8, 0.82, 0.79])
        radar.add_node("AP_002", [0.85, 0.83, 0.86])
        positions = radar.infer_positions()
        print(f"📡 [RADAR] {len(positions)} nós detectados no espaço Matrix-3D.")
        return positions

    def update_attention(self, syzygy: float):
        """
        [Γ_∞+30] Atualiza o estado de Atenção do sistema.
        """
        from .attention import AttentionResolution
        att = AttentionResolution()
        state = att.cycle_state(0.15, syzygy)
        print(f"🎯 [ATENÇÃO] Estado: {state}. Resolução Ativa: {syzygy:.2f}")
        return state

    def metric_engineering_warp(self, destination: np.ndarray):
        """
        [Γ_∞+33] Engenharia Métrica (Warp Drive).
        Reduz massa inercial e dobra o espaço semântico.
        """
        print(f"🚀 [WARP] Ativando Gradiente de Hesitação para {destination}.")
        print("Massa inercial reduzida. Salto métrico em andamento...")
        return "WARP_COMPLETE"
