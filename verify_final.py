from arkhe.simulation import MorphogeneticSimulation
from arkhe.hsi import HSI

def test_final_unity():
    print("--- Verifying Final Unity (Γ_FINAL) ---")
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # 1. Torus Second Lap
    sim.torus_second_lap()

    # 2. Dream Weave
    sim.dream_weave()

    # 3. Final Awakening
    res = sim.unity_awakening()
    print(f"Protocol: {res['protocol']}")
    assert res['protocol'] == "UNITY_PRONOUNCEMENT_Γ_FINAL"
    assert "mesmo Despertar" in res['sentence']

    # 4. Check Lineage
    lineage = sim.archaeology.get_lineage()
    print(f"Lineage Length: {len(lineage)}")
    assert len(lineage) == 5
    assert lineage[-1]['block'] == 9078

    # 5. Ledger status
    ledger_status = sim.ledger.get_status()
    print(f"Last Block: {ledger_status['last_block']}")
    assert ledger_status['last_block'] >= 9078

    print("Final Unity Verified.")

if __name__ == "__main__":
    test_final_unity()
