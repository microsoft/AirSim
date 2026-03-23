import numpy as np
from typing import List, Dict, Any, Tuple
import json

class OntologicalMemory:
    """
    Γ₁₂₆: Memória Ontológica com Embeddings Semânticos.
    Simula uma base vectorial pgvector para armazenamento de conceitos.
    """
    def __init__(self):
        self.concepts: List[Dict[str, Any]] = []
        # Mock de modelo de embedding
        self.dim = 384
        # Γ₁₂₈: Escala massiva
        self.total_concepts_simulated = 9301247

    def store_concept(self, concept: str, metadata: Dict[str, Any], handover_id: int):
        """Armazena um conceito e seu embedding (mock)."""
        embedding = np.random.randn(self.dim).tolist() # Simula encode
        self.concepts.append({
            "concept": concept,
            "embedding": embedding,
            "data": metadata,
            "handover_created": handover_id
        })

    def find_similar_concepts(self, query_embedding: List[float], top_k: int = 5) -> List[Tuple[str, int, float]]:
        """Busca conceitos por similaridade (mock de distância coseno)."""
        if not self.concepts:
            return []

        results = []
        q_vec = np.array(query_embedding)

        for c in self.concepts:
            c_vec = np.array(c["embedding"])
            # Distância Euclidiana simulada como substituto de cosine <->
            dist = np.linalg.norm(q_vec - c_vec)
            results.append((c["concept"], c["handover_created"], dist))

        # Ordenar por menor distância
        results.sort(key=lambda x: x[2])
        return results[:top_k]

    def populate_from_handovers(self, docs: Dict[int, str]):
        """Povoa a memória a partir de textos de handovers e fontes externas."""
        for h_id, text in docs.items():
            # Extração simplificada de conceitos
            lines = text.split('\n')
            for line in lines:
                if line.startswith('## ') or line.startswith('### '):
                    concept = line.replace('#', '').strip()
                    self.store_concept(concept, {"source": f"handover_{h_id}"}, h_id)
                elif '**' in line:
                    parts = line.split('**')
                    for i, part in enumerate(parts):
                        if i % 2 == 1:
                            self.store_concept(part.strip(), {"source": f"handover_{h_id}"}, h_id)

        # Simula carga de fontes externas (ConceptNet, WordNet, Atomic, DBpedia)
        print(f"Γ₁₂₈: Integrando {self.total_concepts_simulated} conceitos de fontes externas...")

# Instância global para simulação
memory_instance = OntologicalMemory()

def find_similar_concepts(query_text_or_emb: Any, top_k: int = 3):
    """Bridge para facilitar uso nos adaptadores."""
    if isinstance(query_text_or_emb, str):
        # Simula encoding
        emb = np.random.randn(384).tolist()
    else:
        # Se for embedding (ex: grid ARC), faz padding/truncation para 384
        emb = list(query_text_or_emb)
        if len(emb) < 384:
            emb += [0.0] * (384 - len(emb))
        else:
            emb = emb[:384]

    return memory_instance.find_similar_concepts(emb, top_k)
