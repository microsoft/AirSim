from dataclasses import dataclass, field
from typing import Tuple, List, Optional
import numpy as np

@dataclass
class CIEF:
    """
    CIEF Genome: Identity functional of a voxel or agent.
    C: Construction / Physicality (Structural properties)
    I: Information / Context (Semantic/Historical data)
    E: Energy / Environment (Thermal/Tension fields)
    F: Function / Frequency (Functional vocation)
    """
    c: float = 0.0
    i: float = 0.0
    e: float = 0.0
    f: float = 0.0

    def to_array(self) -> np.ndarray:
        return np.array([self.c, self.i, self.e, self.f], dtype=np.float32)

@dataclass
class HexVoxel:
    """
    HexVoxel: A unit of the Hexagonal Spatial Index (HSI).
    """
    # Cube coordinates (q, r, s) where q + r + s = 0, plus h for height
    coords: Tuple[int, int, int, int]

    # CIEF Genome
    genome: CIEF = field(default_factory=CIEF)

    # Coherence local (Phi metric)
    phi_data: float = 0.0
    phi_field: float = 0.0
    conflict_level: float = 0.0 # New: for interference detection
    tau: float = 0.0 # New: Entanglement Tension

    @property
    def phi(self) -> float:
        # Integrated coherence adjusted by conflict
        base_phi = (self.phi_data + self.phi_field) / 2.0
        return base_phi * (1.0 - self.conflict_level)

    # Quantum-like state (amplitudes for 6 faces + internal)
    # state[0-5] are faces, state[6] is internal
    state: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.complex64))

    # Reaction-diffusion state (A, B) for Gray-Scott model
    rd_state: Tuple[float, float] = (1.0, 0.0)
    memory_bias: float = 0.0 # M(x) term for conditioned reflex
    stability_index: float = 1.0 # New: S(t) = 1 - |dF/dt|
    is_quarantined: bool = False # New: Byzantine isolation
    sensor_health: float = 1.0 # New: 1.0 = OK, 0.0 = Failed
    rehabilitation_index: float = 0.0 # New: Trust recovery [0, 1]

    # Hebbian weights for 6 neighbors
    weights: np.ndarray = field(default_factory=lambda: np.ones(6, dtype=np.float32))

    def __post_init__(self):
        if len(self.state) != 7:
            self.state = np.zeros(7, dtype=np.complex64)
        if len(self.weights) != 6:
            self.weights = np.ones(6, dtype=np.float32)
