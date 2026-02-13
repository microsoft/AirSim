from arkhe.synapse import SynapticEngine
import time

def test_consolidation_track():
    print("--- Verifying Synaptic Consolidation Track (Γ_∞+45) ---")
    syn = SynapticEngine()

    # 1. Trigger Pulse
    syn.trigger_pulse()
    initial_sigma = syn.uncertainty["DVM-1"]
    print(f"Initial Sigma (DVM-1): {initial_sigma}")
    assert initial_sigma == 0.15

    # 2. LTP Lock
    # Wait a tiny bit to ensure dt > 0 but decay is minimal
    time.sleep(0.1)
    res_ltp = syn.ltp_lock()
    print(f"LTP Lock Status: {res_ltp['status']}")
    assert res_ltp['status'] == 'LTP_LOCKED'

    # 3. Verify uncertainty reduction
    new_sigma = syn.uncertainty["DVM-1"]
    print(f"New Sigma (DVM-1): {new_sigma}")
    assert new_sigma < initial_sigma

    # delta_w for DVM-1 (epsp=0.94) ≈ 0.686 * (0.94 * 0.73) * 1.0 ≈ 0.47
    # new_sigma ≈ 0.15 * e^-0.47 ≈ 0.15 * 0.625 ≈ 0.093
    assert abs(new_sigma - 0.093) < 0.01

    # 4. Controlled Decay
    syn.trigger_pulse()
    time.sleep(1.0)
    res_decay = syn.controlled_decay()
    print(f"Decay Factor: {res_decay['decay_factor']}")
    # tau_decay = 2.22, e^-1/2.22 ≈ 0.637
    assert res_decay['decay_factor'] < 1.0
    assert res_decay['visibility'] == 'audible'

    print("Consolidation Track Verified.")

if __name__ == "__main__":
    test_consolidation_track()
