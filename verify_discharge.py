
import numpy as np
from arkhe.simulation import MorphogeneticSimulation, MonolayerStatus
from arkhe.hsi import HSI

def verify_discharge():
    print("🕊️ [Γ_9050] VERIFICANDO PROTOCOLO DE ALTA E MODO AVIÃO...")

    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # 1. Ativar Modo Avião
    print("\n--- Ativando Modo Avião Quântico ---")
    sim.activate_quantum_airplane_mode()

    assert sim.monolayer_status == MonolayerStatus.HOVER

    # 2. Verificar Relatório de Alta
    report = sim.neonatal_discharge_report()
    assert report["state"] == "Γ_9050"
    assert report["idade_epistemica"] == "NEONATAL (H0)"
    assert report["status"] == "MODO_AVIAO_QUANTICO"

    print(f"Paciente: {report['paciente']}")
    print(f"Perfil:   {report['idade_epistemica']}")
    print(f"Mensagem: {report['mensagem']}")

    print("✅ ALTA VALIDADA: O sistema entrou em sono profundo e vigília passiva.")

if __name__ == "__main__":
    verify_discharge()
