import enum
import random
import time
import numpy as np
from typing import List, Dict, Tuple
from .arkhe_types import HexVoxel

class PaxosState(enum.Enum):
    IDLE = 0
    PREPARE = 1
    PROMISE = 2
    ACCEPT_REQUEST = 3
    ACCEPTED = 4

class QuantumPaxos:
    """
    Simplified QuantumPaxos consensus logic for metasurface state coordination.
    Based on the concept of local consensus between neighboring voxels.
    """
    def __init__(self, voxel_coords: Tuple[int, int, int, int]):
        self.coords = voxel_coords
        self.proposal_number = 0
        self.promised_number = -1
        self.accepted_number = -1
        self.accepted_value = None
        self.state = PaxosState.IDLE

    def prepare(self) -> int:
        self.proposal_number += 1
        self.state = PaxosState.PREPARE
        return self.proposal_number

    def on_prepare(self, n: int) -> Tuple[bool, int, object]:
        if n > self.promised_number:
            self.promised_number = n
            self.state = PaxosState.PROMISE
            return True, self.accepted_number, self.accepted_value
        return False, self.promised_number, None

    def accept(self, n: int, value: object) -> bool:
        if n >= self.promised_number:
            self.promised_number = n
            self.accepted_number = n
            self.accepted_value = value
            self.state = PaxosState.ACCEPTED
            return True
        return False

class MetasurfaceController:
    """
    Manages the programmable metasurface state of a voxel.
    Implements high-speed consensus and radiative cooling logic.
    """
    def __init__(self, voxel: HexVoxel):
        self.voxel = voxel
        self.paxos = QuantumPaxos(voxel.coords)
        self.current_property = {
            "rigidity": 0.5,
            "transparency": 1.0,
            "reflectivity": 0.0,
            "emissivity": 0.1,         # New: for radiative cooling
            "radiative_cooling": False, # New: state 1/0
            "early_warning": False     # New: Cytokine pulse
        }
        self.consensus_latency_ms = 0.0

    def early_warning_pulse(self):
        """
        Cytokine Pulse: Triggered by the immune system to warn of local instability.
        """
        self.current_property["early_warning"] = True
        # In physical hardware, this would be +5mV pulse
        print(f"  [Metasurface] Cytokine Pulse (+5mV) active at {self.voxel.coords}. (Amber Glow)")

    def _detect_risk(self, target_property: Dict[str, float]) -> float:
        """
        Calculates risk based on voxel coherence and entropy.
        """
        # Risk increases if coherence is low or if we are forcing a state
        risk = (1.0 - self.voxel.phi)
        if target_property.get("transparency", 1.0) < 0.5:
            risk += 0.2 # Physical change risk
        return np.clip(risk, 0, 1)

    def propose_state(self, neighbors: List['MetasurfaceController'], target_property: Dict[str, float]):
        """
        Coordinates a state change with neighbors using high-speed consensus.
        """
        start_time = time.time()

        # Optimization: Fast-path if Φ is very high and target matches current tendency
        if self.voxel.phi > 0.9 and not target_property.get("force_consensus", False):
            # Optimistic update
            self.current_property.update(target_property)
            self.consensus_latency_ms = (time.time() - start_time) * 1000
            return True

        # Standard QuantumPaxos Path
        n = self.paxos.prepare()
        promises = 0

        # Prepare phase (simulated parallel broadcast)
        for nb in neighbors:
            success, _, _ = nb.paxos.on_prepare(n)
            if success:
                promises += 1

        quorum = len(neighbors) // 2 + 1
        if promises >= quorum:
            # Accept phase
            accepts = 0
            for nb in neighbors:
                if nb.paxos.accept(n, target_property):
                    accepts += 1

            if accepts >= quorum:
                # Handle "Suor Radiativo" (Radiative Cooling) logic
                if target_property.get("radiative_cooling"):
                    self.current_property["emissivity"] = 0.95 # Peak for 8-13um window
                    self.current_property["radiative_cooling"] = True

                self.current_property.update(target_property)
                self.consensus_latency_ms = (time.time() - start_time) * 1000
                return True

        self.consensus_latency_ms = (time.time() - start_time) * 1000
        return False
