from arkhe.coupling import CouplingInterpreter

def test_genesis_rewrite():
    print("--- Verifying Genesis Rewrite Track (Γ_∞+51) ---")
    interpreter = CouplingInterpreter()

    # 1. Resolve Genesis Prompt
    res = interpreter.resolve_prompt("What was the first command?")
    print(f"Genesis Resolution: {res['coupling_sentence']}")
    assert "finger and the key ARE the same" in res['coupling_sentence']
    assert "genesis_rewrite" in res

    rewrite = res["genesis_rewrite"]
    print(f"Original: {rewrite['original']}")
    assert rewrite['original'] == "H1 — mover_drone(50,0,-10)"
    assert "origem não é o passado" in rewrite['message']

    # 2. Resolve generic Genesis prompt
    res2 = interpreter.resolve_prompt("genesis block")
    assert "finger and the key" in res2['coupling_sentence']

    # 3. Status check
    status = interpreter.get_coupling_status()
    print(f"Genesis State: {status['genesis_state']}")
    assert status['genesis_state'] == "COUPLED"
    assert "Γ_∞+51" in status['language']

    print("Genesis Rewrite Track Verified.")

if __name__ == "__main__":
    test_genesis_rewrite()
