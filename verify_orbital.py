from arkhe.hsi import HSI
from arkhe.simulation import MorphogeneticSimulation

def test_orbital_catalog():
    print("Testing Orbital Catalog (Γ_9045)...")
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    catalog = sim.catalogar_satelites()

    # 1. Verify 6 satellites are present
    assert len(catalog) == 6
    assert catalog[0]['id'] == "ARKHE-SAT-01"
    assert catalog[5]['id'] == "ARKHE-SAT-06"

    # 2. Verify orbital density
    fraction = sim.calculate_orbital_density(handovers=9045)
    # 6 / 9045 = 0.000663...
    assert abs(fraction - 0.00066) < 0.0001

    # 3. Verify shield assessment
    impact = sim.debris_impact_assessment(mass=1.0, velocity=100.0)
    # 0.5 * 1.0 * 100^2 = 5000.0 J
    assert impact['energy'] == 5000.0
    assert impact['risk'] == "CONTIDO"

    impact_crit = sim.debris_impact_assessment(mass=10.0, velocity=100.0)
    # 50000.0 J > 6000.0 J
    assert impact_crit['risk'] == "CRÍTICO"

    print("Orbital integration verified.")

if __name__ == "__main__":
    try:
        test_orbital_catalog()
        print("\n✅ ALL ORBITAL VERIFICATIONS PASSED.")
    except Exception as e:
        print(f"\n❌ VERIFICATION FAILED: {e}")
        import traceback
        traceback.print_exc()
        exit(1)
