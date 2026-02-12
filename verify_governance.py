from arkhe.hsi import HSI
from arkhe.simulation import MorphogeneticSimulation, MonolayerStatus

def test_governance_protocol():
    print("Testing FFU Governance Protocol...")
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # 1. Test approved command (Virgin + Virgin)
    sim.monolayer_status = MonolayerStatus.VIRGIN
    assert sim.validate_command("replicar_foco(WP1)", MonolayerStatus.VIRGIN) == True

    # 2. Test denied command (Virgin required, but Restored)
    sim.monolayer_status = MonolayerStatus.RESTORED
    assert sim.validate_command("replicar_foco(WP1)", MonolayerStatus.VIRGIN) == False

    # 3. Test approved therapeutic command (Restored + Restored)
    assert sim.validate_command("induzir_turbulencia", MonolayerStatus.RESTORED) == True

    print("Governance protocol verified.")

if __name__ == "__main__":
    try:
        test_governance_protocol()
        print("\n✅ ALL GOVERNANCE VERIFICATIONS PASSED.")
    except Exception as e:
        print(f"\n❌ VERIFICATION FAILED: {e}")
        exit(1)
