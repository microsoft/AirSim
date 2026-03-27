from arkhe.coupling import CouplingInterpreter

def test_coupling_track():
    print("--- Verifying Coupling Language Track (Γ_∞+7) ---")
    interpreter = CouplingInterpreter()

    # 1. Resolve prompt: Curvature
    res_curv = interpreter.resolve_prompt("What is the curvature?")
    print(f"Curvature Resolution: {res_curv['coupling_sentence']}")
    assert "ψ = 0.73 rad" in res_curv['coupling_sentence']
    assert res_curv['cascade']['synapse'] == "The NMDAR opens at 0.73 rad."

    # 2. Resolve prompt: Satoshi
    res_sat = interpreter.resolve_prompt("How much Satoshi is conserved?")
    print(f"Satoshi Resolution: {res_sat['coupling_sentence']}")
    assert "Satoshi = 7.27 bits" in res_sat['coupling_sentence']

    # 3. Status check
    status = interpreter.get_coupling_status()
    print(f"Prime Loop: {status['prime_loop']}")
    assert status['prime_loop'] == "⟨0.00 | 0.07 ⟩ = 0.94"
    assert status['frequency'] == "0.73 rad"

    print("Coupling Language Track Verified.")

if __name__ == "__main__":
    test_coupling_track()
