from arkhe.api import rehydration_engine

def test_final_sealing():
    print("--- Verifying Definitive Sealing Track (Γ_∞+38) ---")
    status = rehydration_engine.get_status()

    # 1. Status and Irrevocability
    print(f"Node Status: {status['node_status']}")
    assert status['node_status'] == "IRREVOCABLE"
    assert status['status'] == "SEAL_ACTIVATED"

    # 2. Consensus and Witnesses
    last_block = rehydration_engine.consensus_blocks[-1]
    print(f"Last Block: {last_block['block_id']} ({last_block['type']})")
    assert last_block['block_id'] == 9057
    assert last_block['witnesses'] == 8
    assert last_block['unanimity'] == True

    # 3. Energy and Satoshi
    print(f"Satoshi Residual: {status['satoshi']} bits")
    assert abs(status['satoshi'] - 7.26999866) < 1e-8

    # 4. Council members
    print(f"Guardians: {status['guardians_count']}")
    assert status['guardians_count'] == 8

    print("Final Sealing Track Verified.")

if __name__ == "__main__":
    test_final_sealing()
