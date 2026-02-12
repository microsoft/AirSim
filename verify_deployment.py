from arkhe.hsi import HSI
from arkhe.simulation import MorphogeneticSimulation

def test_deployment_protocol():
    print("Testing Deployment Protocol (Γ_9039)...")
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # 1. Verify 6 stones initialization
    assert len(sim.foci) == 6
    assert "KERNEL" in sim.foci

    # 2. Verify capacity check
    occupancy = sim.check_monolayer_capacity()
    # WP1(0.03) + DVM(0.02) + Bola(0.015) + Identity(0.02) + M1(0.025) + Kernel(0.06) = 0.170
    assert abs(occupancy - 0.170) < 0.001

    # 3. Test place_stone (Formal)
    sim.place_stone("FORMAL", 1000.0)
    assert "FORMAL" in sim.foci

    # 4. Verify increased occupancy
    new_occupancy = sim.check_monolayer_capacity()
    # 0.170 + FORMAL(0.05 default for unknown names) = 0.220
    assert abs(new_occupancy - 0.220) < 0.001

    # 5. Test confirmation method
    report = sim.confirmar_implantacao()
    assert report["state"] == "Γ_9039"

    print("Deployment protocol verified.")

if __name__ == "__main__":
    try:
        test_deployment_protocol()
        print("\n✅ ALL DEPLOYMENT VERIFICATIONS PASSED.")
    except Exception as e:
        print(f"\n❌ VERIFICATION FAILED: {e}")
        import traceback
        traceback.print_exc()
        exit(1)
