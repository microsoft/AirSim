from arkhe.api import rehydration_engine

def test_high_throughput():
    print("--- Verifying High-Throughput Track (Γ_∞+34) ---")
    status = rehydration_engine.get_status()

    # 1. Throughput check
    print(f"Max Throughput: {status['throughput_max']} handovers/s")
    assert status['throughput_max'] == 2000

    # 2. Test History
    print(f"Total Tests: {len(rehydration_engine.throughput_tests)}")
    assert len(rehydration_engine.throughput_tests) >= 1
    test = rehydration_engine.throughput_tests[0]
    assert test['rate'] == 2000
    assert test['latency_ms'] == 0.47

    print("High-Throughput Track Verified.")

if __name__ == "__main__":
    test_high_throughput()
