# encoding.py
# Simulação de encoder CNN para grids ARC (Γ₁₂₈)

import numpy as np
from typing import List

class GridEncoder:
    """
    Γ₁₂₈: Codificação de Grid Rica.
    Utiliza uma estrutura pseudo-convolucional para extrair características espaciais.
    """
    def __init__(self, output_dim: int = 384):
        self.output_dim = output_dim

    def encode(self, grid: List[List[int]]) -> np.ndarray:
        """
        Simula a extração de features convolucionais.
        Em uma implementação real, usaria PyTorch/CNN.
        """
        grid_array = np.array(grid)
        # Simula filtros espaciais (bordas, formas)
        h, w = grid_array.shape
        # Feature: densidade de cores
        color_dist = np.histogram(grid_array, bins=10, range=(0, 9))[0] / (h * w)
        # Feature: simetria horizontal
        sym_h = np.mean(grid_array == np.flip(grid_array, axis=1))
        # Feature: simetria vertical
        sym_v = np.mean(grid_array == np.flip(grid_array, axis=0))

        # Constrói o embedding
        features = np.concatenate([color_dist, [sym_h, sym_v]])
        # Padding/Projection para 384 dim
        embedding = np.zeros(self.output_dim)
        embedding[:len(features)] = features
        # Adiciona ruído determinístico baseado no grid para unicidade
        seed = int(np.sum(grid_array))
        np.random.seed(seed)
        embedding[len(features):] = np.random.randn(self.output_dim - len(features)) * 0.1

        return embedding

encoder_instance = GridEncoder()

def encode_grid(grid: List[List[int]]) -> List[float]:
    return encoder_instance.encode(grid).tolist()
