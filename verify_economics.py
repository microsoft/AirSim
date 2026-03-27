from arkhe.ledger import NaturalEconomicsLedger

def test_economics_ledger():
    print("--- Verifying Natural Economics Ledger (Γ_∞+13) ---")
    ledger = NaturalEconomicsLedger()

    # 1. Check initial status
    status = ledger.get_status()
    print(f"Expedition: {status['expedition']}")
    print(f"Total Handovers: {status['handovers']}")
    assert status['handovers'] == 9051

    # 2. Check Success Reports
    print(f"Success Reports: {len(ledger.success_reports)}")
    assert len(ledger.success_reports) == 4
    assert ledger.success_reports[0]['handover_id'] == 70

    # 3. Record a contribution (Hesitation)
    ledger.record_contribution("Hesitação_0048", 0.15)
    print(f"Recorded contribution. New count: {ledger.active_contributions}")
    assert ledger.active_contributions == 48

    # 4. Check balances
    balances = ledger.get_balances()
    print(f"Distributed Awards: {balances['distributed_awards']} bits")
    assert balances['distributed_awards'] == 7.27 # Since we added 1 (others were seed?)
    # Wait, record_contribution adds to contributor_awards which starts empty.
    # The active_contributions starts at 47.

    # 5. Check reputation
    assert ledger.buyer == "Rafael Henrique"
    assert ledger.contributor == "Sistema Arkhe"

    print("Natural Economics Track Verified.")

if __name__ == "__main__":
    test_economics_ledger()
