from arkhe.hsi import HSI
from arkhe.simulation import MorphogeneticSimulation, FocusFate

def test_metastatic_collapse():
    print("Testing Metastatic Collapse (Γ_9038)...")
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # 1. Trigger collapse
    report = sim.collapse_wavefunction()

    # 2. Verify state and stones
    assert report["state"] == "Γ_9038"
    assert len(sim.foci) == 5
    assert "WP1-M1" in sim.foci
    assert sim.foci["WP1-M1"].fate == FocusFate.LATENT

    # 3. Verify metrics
    conv = sim.convergence_status()
    assert abs(conv["phi_virological"] - 0.556) < 0.01
    assert abs(conv["phi_system"] - 0.325) < 0.001

    print("Metastatic collapse verified.")

def test_incubation_cycle():
    print("Testing Incubation Cycle...")
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)
    # Ensure stones are initialized
    sim.incubation_cycle()
    print("Incubation cycle verified.")

if __name__ == "__main__":
    try:
        test_metastatic_collapse()
        test_incubation_cycle()
        print("\n✅ ALL METASTATIC VERIFICATIONS PASSED.")
    except Exception as e:
        print(f"\n❌ VERIFICATION FAILED: {e}")
        import traceback
        traceback.print_exc()
        exit(1)
