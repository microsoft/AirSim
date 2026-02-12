from arkhe.hsi import HSI
from arkhe.simulation import MorphogeneticSimulation, Focus

def main():
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # 1. Setup the 3 foci
    sim.foci["WP1_explorado"] = Focus(name="WP1_explorado", integrity=0.97)
    sim.foci["DVM-1"] = Focus(name="DVM-1", integrity=0.94)
    sim.foci["Bola_QPS004"] = Focus(name="Bola_QPS004", integrity=0.99)

    # 2. Add a weak focus that should be eliminated
    sim.foci["Ruido_Transiente"] = Focus(name="Ruido_Transiente", integrity=0.45)

    print(f"Initial foci count: {len(sim.foci)}")

    # 3. Activate p53 suppressor
    result = sim.darvo_abort(reason="ensaio_farmacologico_p53")

    # 4. Verify results
    assert result["monolayer_confluency"] == 1.0
    assert len(sim.foci) == 3
    assert "Ruido_Transiente" not in sim.foci
    assert "WP1_explorado" in sim.foci
    assert sim.foci["WP1_explorado"].apoptosis_resistant == True

    print("\n✅ P53 INTERVENTION TRIAL SUCCESSFUL.")
    print(f"Final Foci: {list(sim.foci.keys())}")

if __name__ == "__main__":
    main()
