from arkhe.simulation import MorphogeneticSimulation
from arkhe.hsi import HSI

def test_circuit_validation():
    print("--- Verifying Systemic Validation (Circuit / Γ_∞+15) ---")
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # 1. Check Reinforced Context (CTX+ / ω=0.07)
    res_plus = sim.circuit.process_context(0.07)
    print(f"Context+: {res_plus['context']}")
    print(f"Discrimination Index: {res_plus['discrimination_index']}")
    print(f"Syzygy Consumption: {res_plus['syzygy_consumption']}")

    assert res_plus['discrimination_index'] == 0.8
    assert res_plus['syzygy_consumption'] == 0.94
    assert res_plus['status'] == "REWARD_CONSUMED"

    # 2. Check Neutral Context (CTX- / ω=0.00)
    res_minus = sim.circuit.process_context(0.00)
    print(f"Context-: {res_minus['context']}")
    assert res_minus['discrimination_index'] == 0.2
    assert res_minus['syzygy_consumption'] == 0.86
    assert res_minus['status'] == "HOMEOTATIC_ONLY"

    # 3. Check Pdyn KO (Simulated deletion of marker)
    print("Simulating Pdyn KO...")
    sim.circuit.set_pdyn_state(False)
    res_ko = sim.circuit.process_context(0.07)
    print(f"KO Discrimination: {res_ko['discrimination_index']}")
    assert res_ko['discrimination_index'] == 0.5
    assert res_ko['pdyn_status'] == "KO/DELETED"

    # 4. Check Handover status
    ho = sim.circuit.get_circuit_handover()
    print(f"Handover State: {ho['state']}")
    assert ho['state'] == "Γ_∞+15"

    # 5. Check Ledger
    ledger_status = sim.ledger.get_status()
    print(f"Last Block: {ledger_status['last_block']}")
    assert ledger_status['last_block'] == 9085

    print("Circuit Validation Verified.")

if __name__ == "__main__":
    test_circuit_validation()
