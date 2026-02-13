from arkhe.api import rehydration_engine

def test_governance_track():
    print("--- Verifying Governance Weighting Track (Γ_∞+32) ---")
    status = rehydration_engine.get_status()

    # 1. Reputation Check
    print(f"Reputation FORMAL (Network): {status['reputation_network']}")
    assert status['reputation_network'] == 0.2873

    # 2. Consensus History
    print(f"Total Consensus Blocks: {len(rehydration_engine.consensus_blocks)}")
    assert len(rehydration_engine.consensus_blocks) >= 1
    assert rehydration_engine.consensus_blocks[0]['proposer'] == "FORMAL"

    # 3. Network Size
    print(f"Active Nodes in Governance: {status['active_nodes']}")
    assert status['active_nodes'] == 8

    print("Governance Track Verified.")

if __name__ == "__main__":
    test_governance_track()
