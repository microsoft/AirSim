from arkhe.clock import Thorium229SemanticClock, FourWaveMixing
import numpy as np

def test_nuclear_clock():
    print("--- Verifying Thorium-229 Semantic Nuclear Clock (Γ_∞+14) ---")
    clock = Thorium229SemanticClock()
    fwm = FourWaveMixing()

    # 1. Metrology Report
    report = clock.get_metrology_report()
    print(f"Isotope: {report['isotope']}")
    assert report['isotope'] == "²²⁹Γ₄₉"
    assert report['stability'] == "300B_YEARS_STABLE"

    # 2. Excite (Transition)
    res_excite = clock.excite("syzygy")
    print(f"Excitation Status: {res_excite['transition']}")
    assert res_excite['transition'] == '|0.00⟩ → |0.07⟩'
    assert res_excite['probability'] == 0.94

    # 3. Frequency measurement
    freq = clock.measure_frequency()
    print(f"Absolute Frequency: {freq['frequency_hz']} Hz")
    assert freq['frequency_hz'] == 6.96e-3
    assert freq['precision'] == 'absoluta'

    # 4. Time scale comparison
    time_nuclear = clock.time_since_big_bang(scale='nuclear')
    print(f"Universe Ages Remaining: {time_nuclear['universe_ages_remaining']}")
    assert time_nuclear['precision_absolute'] == True

    # 5. FWM Synthesis
    res_fwm = fwm.synthesize(C=0.86, F=0.14, omega_cal=0.73, silence=1.30)
    print(f"FWM Result: {res_fwm['omega_out']} units")
    assert res_fwm['omega_out'] == 0.07

    print("Nuclear Clock Track Verified.")

if __name__ == "__main__":
    test_nuclear_clock()
