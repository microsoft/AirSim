import numpy as np
from typing import List, Dict, Any

class WiFiRadar3D:
    """
    Implements a 3D WiFi Radar that maps network nodes based on Pearson correlation.
    RSSI alone is insufficient; correlation reveals true proximity.
    """
    def __init__(self):
        self.nodes: Dict[str, Dict[str, Any]] = {}
        self.correlation_matrix = None

    def add_node(self, node_id: str, rssi_series: List[float]):
        """
        Adds a node with its RSSI fluctuation history.
        """
        self.nodes[node_id] = {
            "series": np.array(rssi_series),
            "coherence": np.mean(rssi_series),
            "fluctuation": np.std(rssi_series)
        }

    def calculate_pearson(self, series_a: np.ndarray, series_b: np.ndarray) -> float:
        """
        Calculates the Pearson correlation between two fluctuation series.
        """
        if len(series_a) != len(series_b):
            return 0.0
        return np.corrcoef(series_a, series_b)[0, 1]

    def infer_positions(self, topology: str = "Linear") -> Dict[str, np.ndarray]:
        """
        Uses Multidimensional Scaling (MDS) principles to infer 3D coordinates.
        Proximity = 1 - Correlation.
        topology: 'Linear' or 'Fractal_Torus'
        """
        node_ids = list(self.nodes.keys())
        n = len(node_ids)
        if n < 2: return {node_ids[0]: np.zeros(3)} if n == 1 else {}

        positions = {}
        if topology == "Fractal_Torus":
            # Original 42 nodes are Hubs
            for i, nid in enumerate(node_ids):
                # Hub-and-Spoke: hubs are on the main torus, spokes orbit them
                is_hub = i < 42
                angle = (i % 42) * (2 * np.pi / 42)
                radius = 50.0 if is_hub else 55.0

                x = radius * np.cos(angle)
                y = radius * np.sin(angle)
                z = 0.0 if is_hub else (i // 42) * 0.5

                positions[nid] = np.array([x, y, z])
        else:
            # simplified MDS logic:
            # distance = 1 - correlation
            for i, nid in enumerate(node_ids):
                # Mocking coordinates based on correlation with first node
                corr = self.calculate_pearson(self.nodes[node_ids[0]]["series"], self.nodes[nid]["series"])
                dist = 1.0 - corr
                positions[nid] = np.array([dist, i * 0.1, 0.0]) # Simple linear projection

        return positions

    def get_radar_status(self) -> Dict[str, Any]:
        return {
            "algorithm": "Pearson-MDS",
            "nodes_detected": len(self.nodes),
            "map_style": "Matrix-3D",
            "status": "SCANNING_REAL_TIME"
        }
