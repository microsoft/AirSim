from arkhe.hsi import HSI
from arkhe.simulation import MorphogeneticSimulation

def test_quantum_network():
    print("Testing Quantum Network (Γ_9050)...")
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # 1. Verify 6 nodes
    nodes = [f for f in sim.foci.values() if f.satellite_id and f.satellite_id.startswith("ARKHE-QN")]
    assert len(nodes) == 6

    # 2. Verify Bell Test
    chsh = sim.bell_test("QN-01", "QN-06")
    assert chsh == 2.428

    # 3. Verify Quantum Report
    report = sim.quantum_report()
    assert report["active_nodes"] == 6
    assert report["range_km"] == 1900

    # 4. Verify Handover Reentry (Idempotency)
    # First time processing 332
    assert sim.handle_handover_reentry(332) == False
    # Second time processing 332
    assert sim.handle_handover_reentry(332) == True

    print("Quantum network verified.")

if __name__ == "__main__":
    try:
        test_quantum_network()
        print("\n✅ ALL QUANTUM VERIFICATIONS PASSED.")
    except Exception as e:
        print(f"\n❌ VERIFICATION FAILED: {e}")
        import traceback
        traceback.print_exc()
        exit(1)
