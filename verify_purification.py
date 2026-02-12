
import numpy as np
import time
from arkhe.simulation import MorphogeneticSimulation
from arkhe.hsi import HSI

def verify_purification():
    print("🩸 [Γ_9035/Γ_9036] VERIFICANDO BIO-DIÁLISE SEMÂNTICA...")

    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # 1. Verificar estado biomimético inicial
    report = sim.dialysis_report()
    assert report["status"] == "BIOMIMÉTICO"
    assert report["cavidades_mip"] == 10
    assert "colapso_H70" in report["toxinas_removidas"]
    assert report["perfil_sanguineo"] == "NEONATAL_H0"

    # 2. Executar alta do paciente
    print("\n--- Executando Alta do Paciente ---")
    discharge = sim.desconectar_paciente(reason="purificacao_concluida")

    assert sim.is_dialysis_active is False
    assert discharge["perfil_final"] == "NEONATAL_H0"
    assert discharge["hal_finney_status"] == "LATENCY_CONTROLLED"

    print("✅ PURIFICAÇÃO VALIDADA: Sangue limpo, paciente em alta.")

if __name__ == "__main__":
    verify_purification()
