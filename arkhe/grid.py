import numpy as np
from typing import Dict, Tuple, List, Set, Any

class SpatialHashGrid:
    """
    SpatialHashGrid: Aceleração espacial O(N) para busca de vizinhos.
    """
    def __init__(self, cell_size: float = 2.0):
        self.cell_size = cell_size
        self.grid: Dict[Tuple[int, int, int], List[Any]] = {}

    def _hash(self, position: np.ndarray) -> Tuple[int, int, int]:
        """Converte posição contínua em chave de célula discreta."""
        # Supondo 3D (x, y, z)
        return tuple(np.floor(position / self.cell_size).astype(int))

    def clear(self):
        self.grid.clear()

    def insert(self, agent: Any):
        key = self._hash(agent.position)
        if key not in self.grid:
            self.grid[key] = []
        self.grid[key].append(agent)

    def query_radius(self, position: np.ndarray, radius: float) -> List[Any]:
        """
        Retorna agentes em células dentro do raio.
        """
        center = self._hash(position)
        cells = int(np.ceil(radius / self.cell_size))
        result = []
        for dx in range(-cells, cells + 1):
            for dy in range(-cells, cells + 1):
                for dz in range(-cells, cells + 1):
                    key = (center[0] + dx, center[1] + dy, center[2] + dz)
                    if key in self.grid:
                        result.extend(self.grid[key])
        return result
