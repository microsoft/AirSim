import math
from typing import Tuple, Dict, List
import numpy as np
from .arkhe_types import HexVoxel

class HSI:
    """
    Hexagonal Spatial Index (HSI)
    Manages 3D hexagonal voxels using cube coordinates for the horizontal plane.
    """
    def __init__(self, size: float = 1.0):
        # size is the distance from the center to a corner of the hexagon
        self.size = size
        self.voxels: Dict[Tuple[int, int, int, int], HexVoxel] = {}

    def cartesian_to_hex(self, x: float, y: float, z: float) -> Tuple[int, int, int, int]:
        """
        Converts 3D cartesian coordinates to 3D hexagonal cube coordinates.
        """
        # Horizontal plane conversion (pointy-top hexagons)
        q = (math.sqrt(3)/3 * x - 1/3 * y) / self.size
        r = (2/3 * y) / self.size
        s = -q - r

        # Rounding to nearest hex
        rq, rr, rs = self._cube_round(q, r, s)

        # Vertical axis (h)
        h = int(round(z / (self.size * 2)))

        return (rq, rr, rs, h)

    def hex_to_cartesian(self, q: int, r: int, s: int, h: int) -> Tuple[float, float, float]:
        """
        Converts 3D hexagonal cube coordinates to 3D cartesian coordinates.
        """
        x = self.size * (math.sqrt(3) * q + math.sqrt(3)/2 * r)
        y = self.size * (3/2 * r)
        z = h * (self.size * 2)
        return (x, y, z)

    def _cube_round(self, q: float, r: float, s: float) -> Tuple[int, int, int]:
        rq = int(round(q))
        rr = int(round(r))
        rs = int(round(s))

        q_diff = abs(rq - q)
        r_diff = abs(rr - r)
        s_diff = abs(rs - s)

        if q_diff > r_diff and q_diff > s_diff:
            rq = -rr - rs
        elif r_diff > s_diff:
            rr = -rq - rs
        else:
            rs = -rq - rr

        return (rq, rr, rs)

    def get_voxel(self, coords: Tuple[int, int, int, int]) -> HexVoxel:
        if coords not in self.voxels:
            self.voxels[coords] = HexVoxel(coords=coords)
        return self.voxels[coords]

    def add_point(self, x: float, y: float, z: float, genome_update: Dict[str, float] = None):
        coords = self.cartesian_to_hex(x, y, z)
        voxel = self.get_voxel(coords)
        if genome_update:
            voxel.genome.c += genome_update.get('c', 0)
            voxel.genome.i += genome_update.get('i', 0)
            voxel.genome.e += genome_update.get('e', 0)
            voxel.genome.f += genome_update.get('f', 0)
        return voxel

    def get_neighbors(self, coords: Tuple[int, int, int, int]) -> List[Tuple[int, int, int, int]]:
        q, r, s, h = coords
        directions = [
            (1, -1, 0), (1, 0, -1), (0, 1, -1),
            (-1, 1, 0), (-1, 0, 1), (0, -1, 1)
        ]
        neighbors = []
        for dq, dr, ds in directions:
            neighbors.append((q + dq, r + dr, s + ds, h))
        neighbors.append((q, r, s, h + 1))
        neighbors.append((q, r, s, h - 1))
        return neighbors
