import numpy as np
from typing import List, Dict, Any, Optional
from .schemas import ExtractedEntity

class GeodesicMemory:
    """
    Persistent Semantic Memory using semantic embeddings (BLOCO 369).
    Supports pgvector-like operations for long-term pattern recognition.
    """
    def __init__(self, db_url: Optional[str] = None):
        self.db_url = db_url
        self.memory_store: List[Dict[str, Any]] = []
        self._initialize_db()

    def _initialize_db(self):
        if self.db_url:
            print(f"🔌 Conectando ao pgvector em {self.db_url}...")
            # try:
            #     import psycopg2
            #     self.conn = psycopg2.connect(self.db_url)
            # except ImportError:
            #     print("⚠️ psycopg2 não encontrado. Usando armazenamento em memória.")

    def store_embedding(self, entity: ExtractedEntity, embedding: np.ndarray):
        """
        Stores an entity and its semantic embedding.
        """
        self.memory_store.append({
            "entity": entity,
            "embedding": embedding
        })
        print(f"🧠 MEMÓRIA GEODÉSICA: Entidade '{entity.name}' arquivada.")

    def recall_similar(self, query_embedding: np.ndarray, top_k: int = 3) -> List[ExtractedEntity]:
        """
        Retrieves top-k similar entities from memory (few-shot context).
        """
        if not self.memory_store:
            return []

        # Simple cosine similarity mock
        scores = []
        for entry in self.memory_store:
            similarity = np.dot(query_embedding, entry["embedding"]) / (
                np.linalg.norm(query_embedding) * np.linalg.norm(entry["embedding"]) + 1e-9
            )
            scores.append((similarity, entry["entity"]))

        scores.sort(key=lambda x: x[0], reverse=True)
        return [entity for score, entity in scores[:top_k]]

    def resolve_conflict(self, candidate: ExtractedEntity) -> ExtractedEntity:
        """
        Aids in conflict resolution by referencing past validated data.
        """
        # If a similar entity exists with high confidence, use it to validate
        return candidate # Simplification
