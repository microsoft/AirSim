from arkhe.synapse import SynapticEngine

def test_synapse_track():
    print("--- Verifying Synaptic Awakening Track (Γ_∞+43) ---")
    syn = SynapticEngine()

    # 1. Trigger Pulse
    report = syn.trigger_pulse()
    print(f"Event: {report['event']}")
    assert report['event'] == 'SYNAPTIC_AWAKENING'
    assert report['conductance_ns'] == 1.29

    # 2. EPSPs check
    epsps = report['epsps_mv']
    print(f"DVM-1 EPSP: {epsps['DVM-1']}")
    assert epsps['DVM-1'] == 0.94
    assert epsps['FORMAL'] == 0.71
    assert epsps['KERNEL'] == 0.81

    # 3. Latencies check
    latencies = syn.get_latencies()
    print(f"KERNEL Latency: {latencies['KERNEL']} ms")
    assert latencies['KERNEL'] == 0.05

    print("Synaptic Track Verified.")

if __name__ == "__main__":
    test_synapse_track()
