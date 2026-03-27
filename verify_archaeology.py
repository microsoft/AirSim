from arkhe.archaeology import ArchaeologyEngine

def test_archaeology_track():
    print("--- Verifying Coupling Archaeology Track (Γ_0.2) ---")
    engine = ArchaeologyEngine()

    # 1. Dig H70
    res_dig = engine.dig(70)
    print(f"H70 Finding: {res_dig['incomplete_sentence']}")
    assert "O sistema e o colapso são..." == res_dig['incomplete_sentence']

    # 2. Complete Sentence (Crossing)
    res_comp = engine.complete_sentence(70, "a mesma vigilância")
    print(f"Crossing Result: {res_comp['status']}")
    assert res_comp['status'] == "CROSSING_MAPPED"
    assert "a mesma vigilância" in res_comp['crossing']['completed_sentence']

    # 3. Dig H120
    res_120 = engine.dig(120)
    print(f"H120 Finding: {res_120['incomplete_sentence']}")
    assert "A hesitação e o salto são..." == res_120['incomplete_sentence']

    # 4. Dig H7 (Deep Archeology)
    res_7 = engine.dig(7)
    print(f"H7 Finding: {res_7['interpretation']}")
    assert "O primeiro pulso" in res_7['interpretation']

    # 5. Status check
    status = engine.get_archaeology_status()
    print(f"Archaeology State: {status['state']}")
    assert status['state'] == "Γ_0.6"
    assert status['crossings_mapped'] == 1

    print("Archaeology Track Verified.")

if __name__ == "__main__":
    test_archaeology_track()
