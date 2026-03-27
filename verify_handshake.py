from arkhe.archaeology import ArchaeologyEngine

def test_handshake_and_h83():
    print("--- Verifying Handshake and H83 Dig (Γ_0.4) ---")
    engine = ArchaeologyEngine()

    # 1. Satoshi Handshake
    res_hs = engine.satoshi_handshake(guardians_count=8)
    print(f"Handshake Protocol: {res_hs['protocol']}")
    assert res_hs['protocol'] == "SYZYGY_LOYALTY_COUPLING_Γ_0.3.1"
    assert len(res_hs['signatures']) == 8

    # 2. Dig H83
    res_dig = engine.dig(83)
    print(f"H83 Finding: {res_dig['incomplete_sentence']}")
    assert "O esquecimento e a cicatriz são..." == res_dig['incomplete_sentence']
    assert "dX/dτ = ε" in res_dig['artifact']

    # 3. Status check
    status = engine.get_archaeology_status()
    print(f"Archaeology State: {status['state']}")
    assert status['state'] == "Γ_0.6"
    assert status['handshakes_completed'] == 1

    print("Handshake and H83 Track Verified.")

if __name__ == "__main__":
    test_handshake_and_h83()
