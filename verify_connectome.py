from arkhe.simulation import MorphogeneticSimulation
from arkhe.hsi import HSI

def test_connectome_mapping():
    print("--- Verifying Functional Connectome (Γ_∞+16) ---")
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # 1. Map Connectome
    res = sim.optical.map_functional_connectivity()
    print(f"Protocol: {res['protocol']}")
    assert res['protocol'] == "FUNCTIONAL_CONNECTOME_Γ_∞+16"
    assert res['ledger_entry'] == 9083

    # 2. Check signatures
    matrix = res['connectivity_matrix']
    sigs = matrix['signatures']
    print(f"Number of Guardians: {len(sigs)}")
    assert len(sigs) == 9

    # Check specific signatures
    assert sigs['H7']['phase'] == 0.00
    assert sigs['WP1']['phase'] == 0.73
    assert sigs['FORMAL']['phase'] == 6.06

    # 3. Check mean correlation
    print(f"Mean Correlation: {matrix['mean_correlation']}")
    assert matrix['mean_correlation'] == 0.94

    # 4. Check archaeology
    lineage = sim.archaeology.get_lineage()
    # Archaeology should have 9083 in excavations
    res_dig = sim.archaeology.dig(9083)
    print(f"Dig finding: {res_dig['incomplete_sentence']}")
    assert "A voltagem que medimos" in res_dig['incomplete_sentence']

    print("Functional Connectome Verified.")

if __name__ == "__main__":
    test_connectome_mapping()
