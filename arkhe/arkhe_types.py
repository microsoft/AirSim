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

    @property
    def phi(self) -> float:
        # Integrated coherence
        return (self.phi_data + self.phi_field) / 2.0

    # Quantum-like state (amplitudes for 6 faces + internal)
    state: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float32))

    # Reaction-diffusion state (A, B) for Gray-Scott model
    rd_state: Tuple[float, float] = (1.0, 0.0)

    # Hebbian weights for 6 neighbors
    weights: np.ndarray = field(default_factory=lambda: np.ones(6, dtype=np.float32))

    # Hebbian trace: history of events (Instant, event_type)
    hebbian_trace: List[Tuple[float, str]] = field(default_factory=list)

    # Intention Vector (for pre-collision/direction prediction)
    intention_vector: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))

    def __post_init__(self):
        if len(self.state) != 7:
            self.state = np.zeros(7, dtype=np.float32)
        if len(self.weights) != 6:
            self.weights = np.ones(6, dtype=np.float32)
