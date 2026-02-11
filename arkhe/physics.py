import numpy as np
from typing import Dict, Tuple, List, Set

class SpatialHashGrid:
    """
    Aceleração Espacial: Neighbor lookups em O(1).
    """
    def __init__(self, cell_size: float = 3.0):
        self.cell_size = cell_size
        self.grid: Dict[Tuple[int, int, int], Set[int]] = {}

    def _hash(self, position: np.ndarray) -> Tuple[int, int, int]:
        return tuple(np.floor(position / self.cell_size).astype(int))

    def clear(self):
        self.grid.clear()

    def insert(self, agent_id: int, position: np.ndarray):
        key = self._hash(position)
        if key not in self.grid:
            self.grid[key] = set()
        self.grid[key].add(agent_id)

    def query_radius(self, position: np.ndarray, radius: float) -> Set[int]:
        """Busca vizinhos dentro de um raio usando o grid hash."""
        center = self._hash(position)
        cells = int(np.ceil(radius / self.cell_size))
        result = set()
        for dx in range(-cells, cells + 1):
            for dy in range(-cells, cells + 1):
                for dz in range(-cells, cells + 1):
                    key = (center[0] + dx, center[1] + dy, center[2] + dz)
                    if key in self.grid:
                        result.update(self.grid[key])
        return result

def calculate_collision_probability(pos_a, vel_a, pos_b, vel_b, dt: float = 0.1) -> float:
    """
    Calcula P_coll derivada da proximidade e velocidade relativa.
    P_coll = exp(- (r . v_rel) / |v_rel|^2) se em rota de interceptação.
    """
    r = pos_b - pos_a
    v_rel = vel_a - vel_b

    dist = np.linalg.norm(r)
    if dist < 0.1: return 1.0 # Já colidindo

    # Projeção da velocidade relativa no vetor posição
    approach_speed = np.dot(v_rel, r / dist)

    if approach_speed <= 0:
        return 0.0 # Se afastando ou paralelo

    # Tempo estimado até colisão
    t_coll = dist / approach_speed

    # Probabilidade decai com o tempo até colisão
    prob = np.exp(-t_coll / dt)
    return np.clip(prob, 0.0, 1.0)
