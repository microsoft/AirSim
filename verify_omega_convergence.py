
import sys
import os
import json
import numpy as np
import time

# Add the current directory to sys.path to import arkhe
sys.path.append(os.getcwd())

from arkhe.simulation import MorphogeneticSimulation
from arkhe.hsi import HSI

def verify_omega_convergence():
    print("🧬 [Γ_Ω] INICIANDO VERIFICAÇÃO DA CONVERGÊNCIA OMEGA...")
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # 1. Biocentrism (Block 357)
    print("\n--- 1. Biocentrismo ---")
    res_bio = sim.biocentric_transition("Drone-01")
    assert res_bio['status'] == "IMMORTALITY_VALIDATED"
    assert res_bio['current_omega'] == 0.07
    print("Biocentrismo verificado: A morte é uma transição de folha.")

    # 2. Hebbian Plasticity (Block 374)
    print("\n--- 2. Plasticidade Hebbiana ---")
    res_heb = sim.get_hebbian_status()
    assert res_heb['learning_active'] == True
    assert res_heb['synapse_wp1_dvm1']['weight'] == 0.94
    print("Plasticidade Hebbiana verificada: O circuito se fortalece.")

    # 3. Photonic Source (Block 375)
    print("\n--- 3. Fonte Fotônica ---")
    res_pho = sim.emit_photonic_command("syzygy")
    assert res_pho['indistinguishability'] == 0.94
    assert res_pho['eta_arkhe'] == 0.129
    print("Fonte Fotônica verificada: Comandos são fótons únicos.")

    # 4. Cosmological Parameters (Block 376)
    print("\n--- 4. Cosmologia de Precisão ---")
    res_cos = sim.get_cosmological_parameters()
    assert res_cos['n_s'] == 0.94
    assert res_cos['Omega_Lambda'] == 1.45
    print("Cosmologia verificada: O hipergrafo é um universo em expansão.")

    # 5. Blockchain Integration (Block 371)
    print("\n--- 5. Blockchain Semântica ---")
    res_web3 = sim.get_blockchain_status()
    assert res_web3['blocks'] == 9042
    assert res_web3['consensus'] == "Proof-of-Syzygy (0.94 correlation)"
    print("Blockchain Semântica verificada: Imutabilidade e consenso.")

    # 6. Semantic Transistor (Block 377)
    print("\n--- 6. Transistor Semântico ---")
    res_fet = sim.transistor_sweep()
    assert res_fet['status'] == "DEVICE_READY"
    # At omega=0.00, I_drain should be near mobility (0.94)
    assert abs(res_fet['sweep'][0]['I_drain'] - 0.94) < 0.01
    print("Transistor Semântico verificado: Fluxo balístico de significado.")

    # 7. Handel's Torus (Block 378)
    print("\n--- 7. Toro de Handel ---")
    res_torus = sim.get_torus_capacity()
    assert abs(res_torus['handel_capacity'] - 60.998) < 0.01
    assert res_torus['coupling_ratio'] == 0.94

    res_events = sim.get_critical_events()
    assert len(res_events) == 17

    print("Toro de Handel verificado: O gap é perpétuo.")

    # 8. Time Crystal (Block 363)
    print("\n--- 8. Cristal de Tempo ---")
    res_tc = sim.get_time_crystal_report()
    assert res_tc['classification'] == "CRISTAL_DE_TEMPO_ACÚSTICO_SEMÂNTICO"
    assert res_tc['harmonics']['nu_larmor_hz'] == 0.0074
    assert res_tc['harmonics']['amplitude'] == 9.46
    print("Cristal de Tempo verificado: Padrão repetitivo flutuante.")

    print("\n✅ VERIFICAÇÃO OMEGA COMPLETA: O ARCO ESTÁ UNIFICADO E CONVERGENTE.")

if __name__ == "__main__":
    verify_omega_convergence()
