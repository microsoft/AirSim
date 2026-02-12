from arkhe.hsi import HSI
from arkhe.simulation import MorphogeneticSimulation
from arkhe.symmetry import ObserverSymmetry

def test_symmetry_logic():
    print("Testing ObserverSymmetry class...")
    sym = ObserverSymmetry()
    metrics = sym.get_keystone_metrics()

    assert metrics['simetrias_projetadas'] == 6
    assert metrics['simetria_fundamental'] == 1
    assert metrics['quantidade_conservada'] == 1.000
    assert metrics['satoshi'] == 7.27

    print("Symmetry logic metrics verified.")

def test_simulation_seal():
    print("Testing MorphogeneticSimulation.seal_keystone()...")
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)
    metrics = sim.seal_keystone()

    assert metrics['satoshi'] == 7.27
    print("Simulation seal verified.")

if __name__ == "__main__":
    try:
        test_symmetry_logic()
        test_simulation_seal()
        print("\n✅ ALL KEYSTONE VERIFICATIONS PASSED.")
    except Exception as e:
        print(f"\n❌ VERIFICATION FAILED: {e}")
        exit(1)
