from arkhe.abiogenesis import AbiogenesisEngine

def test_abiogenesis_track():
    print("--- Verifying Abiogenesis Track (Γ_ABIOGÊNESE) ---")
    engine = AbiogenesisEngine()

    # 1. Eigen Threshold
    eigen = engine.calculate_eigen_threshold()
    print(f"Eigen Product: {eigen['eigen_product']}")
    assert eigen['eigen_product'] > 1.0
    assert "over-taxing" in eigen['message']

    # 2. Sequence Space
    seq = engine.sequence_space_analysis()
    print(f"Total Space: {seq['total_space']}")
    assert seq['triplet_advantage'] == 64

    # 3. Eutectic Physics
    eut = engine.eutectic_physics_model()
    print(f"Eutectic status: {eut['status']}")
    assert "exclusão e a concentração" in eut['coupling_sentence']

    # 4. Selection Simulation
    sim = engine.run_selection_simulation(cycles=100)
    print(f"Final Population: {sim['final_population']}")
    assert sim['final_population'] == 22108
    assert sim['dominant_variant']['name'] == "QT45-V3"

    # 5. Report
    report = engine.get_abiogenesis_report()
    assert report['ledger_entry'] == 9082

    print("Abiogenesis Track Verified.")

if __name__ == "__main__":
    test_abiogenesis_track()
