from arkhe.api import rehydration_engine

def test_sealing_track():
    print("--- Verifying Final Sealing Track (Γ_∞+36) ---")
    status = rehydration_engine.get_status()

    # 1. Guardians council
    print(f"Total Guardians: {status['guardians_count']}")
    assert status['guardians_count'] == 8

    # 2. Signature verification
    print(f"Assigned Signature: {status['signature']}")
    assert status['signature'] == "SIG_FORMAL_001"

    # 3. Last consensus block
    last_block = rehydration_engine.consensus_blocks[1] # Step 18 block
    print(f"Naming Block: {last_block['block_id']}")
    assert last_block['block_id'] == 9056
    assert last_block['type'] == "NOMEACAO_DE_GUARDIAO"

    print("Sealing Track Verified.")

if __name__ == "__main__":
    test_sealing_track()
