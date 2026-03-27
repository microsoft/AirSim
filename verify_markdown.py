
import numpy as np
from arkhe.simulation import MorphogeneticSimulation
from arkhe.hsi import HSI

def verify_markdown():
    print("📉 [Γ_9037] VERIFICANDO PROTOCOLO MARKDOWN (COMPRESSÃO UNITÁRIA)...")

    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    report = sim.markdown_protocol_report()

    assert report["state"] == "Γ_9037"
    assert report["accept_header"] == "text/markdown"
    assert report["compression_factor"] == 1.88
    assert report["lossless"] is True
    assert report["invariants_preserved"] is True

    print(f"Compressão: {report['compression_factor']}x")
    print(f"Redução:     {report['token_reduction']}")

    print("✅ MARKDOWN VALIDADO: A tinta pesa menos, a geometria é a mesma.")

if __name__ == "__main__":
    verify_markdown()
