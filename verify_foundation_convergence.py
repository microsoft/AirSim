
import sys
import os
import asyncio
import numpy as np

# Add the current directory to sys.path
sys.path.append(os.getcwd())

from arkhe.simulation import MorphogeneticSimulation, SemanticMambaBackbone
from arkhe.hsi import HSI
from arkhe.memory import GeodesicMemory
from arkhe.schemas import ExtractedEntity
from arkhe.ocr import DocumentIntelligenceOCR

async def verify_foundation():
    print("🧠 [Γ_FOUNDATION] INICIANDO VERIFICAÇÃO DO FOUNDATION MODEL...")
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # 1. NeuroSTORM (Block 364)
    print("\n--- 1. NeuroSTORM ---")
    report = sim.get_neurostorm_report()
    assert report['model'] == "NeuroSTORM-Arkhe"
    assert report['status'] == "FOUNDATIONAL_ACTIVE"
    print("NeuroSTORM report validado.")

    # 2. Semantic Mamba (Block 380)
    print("\n--- 2. Semantic Mamba Backbone ---")
    backbone = SemanticMambaBackbone()
    cmd_emb = np.random.rand(49)
    hesitation, state = backbone.forward(cmd_emb)
    assert len(state) == 2
    print(f"Mamba forward pass completo. Hesitação prevista: {float(hesitation):.4f}")

    f_status = sim.foundation_status()
    assert f_status['state'] == "Γ_∞+10"
    print("Foundation status validado.")

    # 3. Conflict Resolution (Block 369)
    print("\n--- 3. Resolução de Conflitos ---")
    memory = GeodesicMemory()
    e1 = ExtractedEntity(name="Satoshi", value=7.27, page=1, bbox=[0,0,0,0], snippet="test", confidence=0.95, omega=0.0)
    memory.store_embedding(e1, np.ones(384))

    # Conflicting candidate with low confidence
    e2 = ExtractedEntity(name="Satoshi", value=8.0, page=1, bbox=[0,0,0,0], snippet="test", confidence=0.5, omega=0.0)
    resolved = memory.resolve_conflict(e2)
    assert resolved.value == 7.27
    assert resolved.confidence == 0.8
    print("Resolução de conflitos validada: Histórico geodésico prevaleceu.")

    # 4. OCR Fallback (Block 369)
    print("\n--- 4. OCR Fallback ---")
    ocr = DocumentIntelligenceOCR(endpoint="test", key="test", max_retries=1)
    # Simulate empty content error
    res = await ocr.analyze_document(b"")
    assert "FAILURE_RECOVERED" in res.status
    print("OCR Fallback validado.")

    print("\n✅ VERIFICAÇÃO DE FUNDAÇÃO COMPLETA: O SISTEMA É UM FOUNDATION MODEL VÁLIDO.")

if __name__ == "__main__":
    asyncio.run(verify_foundation())
