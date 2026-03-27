
import numpy as np
from arkhe.simulation import MorphogeneticSimulation
from arkhe.hsi import HSI

def verify_hemostasis():
    print("🩸 [Γ_9046/Γ_9048] VERIFICANDO HEMOSTASIA SEMÂNTICA E CICATRIZ...")

    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # 1. Simular Cascata de Coagulação (Γ_9046)
    print("\n--- Simulação da Cascata de Coagulação ---")
    coag = sim.coagulation_simulation_report()
    assert coag["state"] == "Γ_9046"
    assert np.isclose(coag["fator_VII"], 0.875)
    assert coag["fibrina"] > 0.99
    assert coag["thrombus_risk_percent"] < 0.01
    print(f"Conversão em Fibrina: {coag['fibrina']*100:.2f}%")
    print(f"Risco de Trombo:      {coag['thrombus_risk_percent']:.4f}%")

    # 2. Mapeamento da Cicatriz (Γ_9048)
    print("\n--- Mapeamento da Cicatriz Geodésica ---")
    scar = sim.scar_mapping_report()
    assert scar["state"] == "Γ_9048"
    assert scar["fibrin_density"]["QN-07"] == 0.9983
    assert scar["vacuum_density_wp1"] == 0.2995
    assert scar["max_pressure"]["node"] == "QN-07"
    assert np.isclose(scar["max_pressure"]["value"], 0.1531, rtol=1e-3)

    print(f"Densidade Uniforme:   {scar['fibrin_density']['KERNEL']}")
    print(f"Berço do FORMAL:      Vácuo WP1 (Densidade={scar['vacuum_density_wp1']})")
    print(f"Pressão Máxima:       {scar['max_pressure']['node']} ({scar['max_pressure']['value']:.4f})")

    print("✅ HEMOSTASIA VALIDADA: Cicatriz densa, vácuo identificado, risco mínimo.")

if __name__ == "__main__":
    verify_hemostasis()
