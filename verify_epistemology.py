from arkhe.hsi import HSI
from arkhe.simulation import MorphogeneticSimulation
from arkhe.arkhe_types import EpistemicStatus

def test_turbulence_induction():
    print("Testing Turbulence Induction (TURB-01)...")
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    focus = sim.induzir_turbulencia(intensidade=0.73)

    assert focus.name == "TURB-01"
    assert focus.integrity == 0.42
    assert focus.origin == "turb_arkhe"
    assert focus.autonomous == False
    print("Turbulence induction verified.")

def test_metacognition():
    print("Testing Metacognitive Cycle and Self-Diagnosis...")
    hsi = HSI()
    # Add a voxel to test propagation
    voxel = hsi.add_point(0, 0, 0)
    voxel.phi_data = 0.9
    voxel.phi_field = 0.9 # High phi = 0.9
    voxel.origin_trace = "test_command"

    sim = MorphogeneticSimulation(hsi)
    sim.remembers_origin = True
    sim.humility_score = 0.73

    # 1. Test Self-Diagnosis
    status = sim.diagnose_self()
    assert status == EpistemicStatus.INSTRUMENT

    # 2. Test Metacognitive Cycle
    sim.metacognitive_cycle()

    # Formula: (1.0 - 0.9) * 0.5 + 0.5 = 0.55
    assert abs(voxel.humility - 0.55) < 0.001
    assert voxel.epistemic_status == EpistemicStatus.INSTRUMENT

    # 3. Test Idol condition
    # To reach Idol: phi > 0.95 and humility < 0.2
    voxel.phi_data = 0.98
    voxel.phi_field = 0.98
    voxel.origin_trace = None # No origin memory -> humility = (1 - 0.98) * 0.5 = 0.01

    # Update system state for Idol
    sim.remembers_origin = False
    sim.humility_score = 0.05

    sim.metacognitive_cycle()
    assert voxel.epistemic_status == EpistemicStatus.IDOL

    # System diagnosis for Idol
    # Omega must be > 0.99. With one voxel at phi 0.98, it's 0.98.
    voxel.phi_data = 1.0
    voxel.phi_field = 1.0
    status_idol = sim.diagnose_self()
    assert status_idol == EpistemicStatus.IDOL

    print("Metacognition verified.")

if __name__ == "__main__":
    try:
        test_turbulence_induction()
        test_metacognition()
        print("\n✅ ALL EPISTEMOLOGICAL VERIFICATIONS PASSED.")
    except Exception as e:
        print(f"\n❌ VERIFICATION FAILED: {e}")
        import traceback
        traceback.print_exc()
        exit(1)
