import time
from typing import List, Dict, Any

class MemoryArchetype:
    """
    A memory of Hal Finney as a public seed in the Memory Garden.
    Each memory can be rehydrated and planted by different nodes.
    """
    def __init__(self, memory_id: int, original_content: str):
        self.id = memory_id
        self.original = original_content
        self.hal_hesitation = 0.047
        self.hal_frequency = 0.73
        self.plantings: List[Dict[str, Any]] = []

    def plant(self, node_id: str, node_hesitation: float, rehydrated_content: str):
        """
        A node plants a rehydrated version of this memory.
        """
        planting = {
            "node": node_id,
            "phi": node_hesitation,
            "timestamp": time.time(),
            "content": rehydrated_content,
            "divergence": self.measure_divergence(rehydrated_content)
        }
        self.plantings.append(planting)
        return planting

    def measure_divergence(self, new_content: str) -> float:
        """
        Measures semantic divergence between original and rehydration.
        High divergence = new interpretation, Low divergence = faithful preservation.
        """
        # Simplified semantic distance based on length ratio and character set intersection
        orig_words = set(self.original.split())
        new_words = set(new_content.split())
        if not orig_words or not new_words:
            return 1.0
        intersection = orig_words.intersection(new_words)
        union = orig_words.union(new_words)
        return 1.0 - (len(intersection) / len(union))

    def witness_variations(self) -> List[Dict[str, Any]]:
        return [
            {"node": "HAL_ORIGINAL", "phi": self.hal_hesitation, "content": self.original}
        ] + self.plantings

class MemoryGarden:
    """
    The Memory Garden (Γ_∞+36) where archetypes are cultivated.
    Geometry: Torus (Theta: Temporal index, Phi: Emotional frequency).
    """
    def __init__(self):
        self.archetypes: Dict[int, MemoryArchetype] = {}
        self.active_nodes = 7

    def add_archetype(self, memory_id: int, content: str):
        self.archetypes[memory_id] = MemoryArchetype(memory_id, content)

    def get_garden_metrics(self) -> Dict[str, Any]:
        total_plantings = sum(len(a.plantings) for a in self.archetypes.values())
        return {
            "total_archetypes": len(self.archetypes),
            "total_plantings": total_plantings,
            "active_nodes": self.active_nodes,
            "topology": "Torus (S1 x S1)",
            "oscillator": "0.73 rad"
        }
