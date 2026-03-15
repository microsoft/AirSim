import numpy as np
from typing import List, Dict, Tuple, Set
from .hsi import HSI
from .arkhe_types import HexVoxel

class HumanPerspectiveEngine:
    """
    Translates raw HSI data into a human-like perspective of objects and context.
    Groups voxels into meaningful 'entities' based on CIEF similarity and proximity.
    """
    def __init__(self, hsi: HSI):
        self.hsi = hsi
        self.objects: Dict[int, List[Tuple[int, int, int, int]]] = {}
        self.next_obj_id = 0

    def identify_objects(self, similarity_threshold: float = 0.85):
        """
        Segment the HSI into objects.
        A human perspective sees a collection of voxels as a single 'Car' or 'Wall'.
        """
        visited: Set[Tuple[int, int, int, int]] = set()
        self.objects = {}

        for coords, voxel in self.hsi.voxels.items():
            if coords in visited or voxel.genome.c < 0.1: # Skip empty or visited
                continue

            # Start new object grouping (Flood Fill / BFS)
            obj_id = self.next_obj_id
            self.next_obj_id += 1
            self.objects[obj_id] = []

            queue = [coords]
            visited.add(coords)

            while queue:
                current = queue.pop(0)
                self.objects[obj_id].append(current)
                curr_voxel = self.hsi.voxels[current]

                neighbors = self.hsi.get_neighbors(current)
                for nb in neighbors:
                    if nb in self.hsi.voxels and nb not in visited:
                        nb_voxel = self.hsi.voxels[nb]

                        # Similarity check: are they part of the same material/construct?
                        sim = self._calculate_similarity(curr_voxel, nb_voxel)
                        if sim > similarity_threshold:
                            visited.add(nb)
                            queue.append(nb)

        print(f"  [Human Perspective] Identified {len(self.objects)} distinct objects in the terrain.")
        self._assign_labels()

    def _calculate_similarity(self, v1: HexVoxel, v2: HexVoxel) -> float:
        # Distance between CIEF genomes
        g1 = v1.genome.to_array()
        g2 = v2.genome.to_array()
        dist = np.linalg.norm(g1 - g2)
        return 1.0 / (1.0 + dist)

    def _assign_labels(self):
        """
        Heuristic-based object labeling to simulate human recognition.
        """
        for obj_id, coords_list in self.objects.items():
            # Calculate average CIEF for the object
            avg_cief = np.mean([self.hsi.voxels[c].genome.to_array() for c in coords_list], axis=0)
            c, i, e, f = avg_cief

            label = "Unknown"
            if c > 0.8 and e > 0.3 and i > 0.3: label = "Vehicle"
            elif c > 0.3 and e > 0.7: label = "Pedestrian" # High heat, low construction
            elif c > 0.7 and i > 0.4: label = "Structure"
            elif c < 0.5 and i > 0.7: label = "Signal Node"
            elif e > 0.8 and c < 0.2: label = "Thermal Hazard"

            # Special case for 'Pedestre 12'
            if label == "Pedestrian" and obj_id == 12:
                label = "Pedestre 12"

            # Apply label to all voxels in the object
            for coords in coords_list:
                self.hsi.voxels[coords].object_label = label

    def get_contextual_summary(self) -> str:
        """
        Returns a human-readable summary of the scene context.
        """
        summary = []
        counts = {}
        for obj_id, coords in self.objects.items():
            label = self.hsi.voxels[coords[0]].object_label
            counts[label] = counts.get(label, 0) + 1

        for label, count in counts.items():
            summary.append(f"{count} {label}(s)")

        return "Terrain Context: " + ", ".join(summary)
