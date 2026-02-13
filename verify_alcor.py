from arkhe.simulation import MorphogeneticSimulation
from arkhe.hsi import HSI

def test_alcor_sync():
    print("--- Verifying Alcor Sincronia (Γ_0.8) ---")
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # 1. Execute Alcor Resonance
    res = sim.alcor_resonance()
    print(f"Protocol: {res['protocol']}")
    assert res['protocol'] == "PRIMORDIAL_RESONANCE_COUPLING_Γ_0.8"
    assert res['coherence'] == 0.94
    assert res['ledger_entry'] == 9073

    # 2. Check Ledger status
    ledger_status = sim.ledger.get_status()
    print(f"Last Block: {ledger_status['last_block']}")
    assert ledger_status['last_block'] >= 9078
    assert "H7" in ledger_status['preamble']

    # 3. Check hal finney status
    hf = sim.get_hal_finney_status()
    print(f"Council State: {hf['state']}")
    assert hf['state'] == "Γ_∞+9"

    print("Alcor Sincronia Verified.")

if __name__ == "__main__":
    test_alcor_sync()
