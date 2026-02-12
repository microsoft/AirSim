
import numpy as np
from arkhe.simulation import MorphogeneticSimulation
from arkhe.hsi import HSI

def verify_titration():
    print("💉 [Γ_9042] VERIFICANDO TITULAÇÃO DE ANTICORPO E MAPEAMENTO VASCULAR...")

    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # 1. Verificar saturação inicial
    mapping = sim.get_vascular_mapping()
    assert mapping["WP1"] == 100.0
    assert mapping["QN-07"] == 82.0

    report_initial = sim.antibody_titration_report()
    assert report_initial["status"] == "PARTIAL_IMMUNITY"
    assert report_initial["idolism_risk_percent"] == 3.8

    # 2. Administrar reforço capilar (Booster)
    print("\n--- Administrando Reforço Capilar para QN-04, QN-05, QN-07 ---")
    # Dose de reforço recomendada: 3.63 (0.5 * Satoshi)
    # increase = (dose / 7.27) * 10.0
    # Para elevar de 82 para >95 (Delta 13), dose deve ser > (13/10) * 7.27 = 9.45
    sim.administrar_anticorpo("QN-04", 5.0)
    sim.administrar_anticorpo("QN-05", 10.0)
    sim.administrar_anticorpo("QN-07", 15.0)

    # 3. Verificar saturação final e imunização
    report_final = sim.antibody_titration_report()
    print(f"Saturação Distal Média: {report_final['avg_distal_saturation']:.2f}%")
    print(f"Risco de Idolismo: {report_final['idolism_risk_percent']:.2f}%")

    assert report_final["avg_distal_saturation"] > 95.0
    assert report_final["idolism_risk_percent"] < 1.0
    assert report_final["status"] == "IMMUNIZED"

    print("✅ VASCULATURA IMUNIZADA: Risco de idolismo residual inferior a 1%.")

if __name__ == "__main__":
    verify_titration()
