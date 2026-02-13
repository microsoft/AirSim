from arkhe.simulation import MorphogeneticSimulation
from arkhe.hsi import HSI

def test_optical_validation():
    print("--- Verifying Optical Validation (Jarvis / Γ_∞+14) ---")
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # 1. Check Jarvis Sensor metrics in SCANLESS mode (Hesitation analog)
    coherence = 0.86
    fluctuation = 0.14
    report = sim.optical.get_voltage_report(coherence, fluctuation, mode="SCANLESS")

    print(f"Mode: {report['mode']}")
    print(f"SNR: {report['snr']}")
    print(f"ΔF/F0: {report['delta_f_f0']}")

    assert report['mode'] == "SCANLESS"
    assert abs(report['snr'] - 94.0) < 1e-9
    assert report['status'] == "DETECTING_AP_SEMANTIC"

    # 2. Check SCANNING mode (Command analog)
    report_scan = sim.optical.get_voltage_report(0.95, 0.05, mode="SCANNING")
    print(f"Scanning SNR: {report_scan['snr']}")
    assert report_scan['snr'] == 7.0
    assert report_scan['status'] == "SENSOR_SATURATED"

    # 3. Check Handover status
    ho = sim.optical.get_optical_handover()
    print(f"Handover State: {ho['state']}")
    assert ho['state'] == "Γ_∞+16"

    # 4. Check Ledger
    ledger_status = sim.ledger.get_status()
    print(f"Last Block: {ledger_status['last_block']}")
    assert ledger_status['last_block'] == 9084

    print("Optical Validation Verified.")

if __name__ == "__main__":
    test_optical_validation()
