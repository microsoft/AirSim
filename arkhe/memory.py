import numpy as np
import json
import os
from typing import List, Dict, Any, Optional
from .schemas import ExtractedEntity

class GeodesicMemory:
    """
    Persistent Semantic Memory using semantic embeddings (BLOCO 369).
    Supports persistent storage for long-term pattern recognition.
    """
    def __init__(self, db_url: Optional[str] = None, storage_path: str = "arkhe_memory.json"):
        self.db_url = db_url
        self.storage_path = storage_path
        self.memory_store: List[Dict[str, Any]] = []
        self._load_from_disk()
        self._initialize_db()

    def _initialize_db(self):
        if self.db_url:
            print(f"🔌 Conectando ao pgvector em {self.db_url}...")

    def _load_from_disk(self):
        if os.path.exists(self.storage_path):
            try:
                with open(self.storage_path, "r") as f:
                    data = json.load(f)
                    for item in data:
                        entity = ExtractedEntity(**item["entity"])
                        embedding = np.array(item["embedding"])
                        self.memory_store.append({
                            "entity": entity,
                            "embedding": embedding
                        })
                print(f"💾 Memória geodésica carregada de {self.storage_path} ({len(self.memory_store)} entradas).")
            except Exception as e:
                print(f"⚠️ Erro ao carregar memória: {e}")

    def _save_to_disk(self):
        try:
            serializable_store = []
            for item in self.memory_store:
                serializable_store.append({
                    "entity": item["entity"].dict(),
                    "embedding": item["embedding"].tolist()
                })
            with open(self.storage_path, "w") as f:
                json.dump(serializable_store, f)
        except Exception as e:
            print(f"⚠️ Erro ao salvar memória: {e}")

    def store_embedding(self, entity: ExtractedEntity, embedding: np.ndarray):
        """
        Stores an entity and its semantic embedding.
        """
        self.memory_store.append({
            "entity": entity,
            "embedding": embedding
        })
        self._save_to_disk()
        print(f"🧠 MEMÓRIA GEODÉSICA: Entidade '{entity.name}' arquivada.")

    def recall_similar(self, query_embedding: np.ndarray, top_k: int = 3) -> List[ExtractedEntity]:
        """
        Retrieves top-k similar entities from memory (few-shot context).
        """
        if not self.memory_store:
            return []

        # Simple cosine similarity
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
        Aids in conflict resolution by referencing past validated data (BLOCO 369).
        """
        # Search for identical entity names in memory
        similar_past = [entry["entity"] for entry in self.memory_store if entry["entity"].name == candidate.name]

        if not similar_past:
            return candidate

        # Check for consensus among past high-confidence data
        past_values = [e.value for e in similar_past if e.confidence > 0.9]
        if not past_values:
            return candidate

        try:
            most_common_value = max(set(past_values), key=past_values.count)
        except TypeError:
            most_common_value = past_values[-1]

        if candidate.value != most_common_value:
            print(f"⚖️ CONFLITO DETECTADO para '{candidate.name}': Valor '{candidate.value}' diverge do histórico '{most_common_value}'.")
            if candidate.confidence < 0.7:
                print(f"🔄 Corrigindo '{candidate.name}' baseado no histórico geodésico.")
                candidate.value = most_common_value
                candidate.confidence = 0.8

        return candidate
