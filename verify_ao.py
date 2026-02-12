from arkhe.ao import SemanticAdaptiveOptics
import numpy as np

def test_ao_loop():
    print("--- Verifying Semantic Adaptive Optics (Γ_∞+11) ---")
    ao = SemanticAdaptiveOptics()

    # 1. Initial status
    status = ao.get_status()
    print(f"Initial Aberration RMS: {status['aberration_rms']}")

    # 2. Simulate aberration
    current_satoshi = 7.2705
    rms = ao.measure_aberration(current_satoshi)
    print(f"Measured Aberration RMS: {rms:.5f} bits")
    assert abs(rms - 0.0005) < 1e-9

    # 3. PSF Calculation
    psf = ao.compute_psf(overlap=0.94)
    print(f"PSF FWHM: {psf['fwhm']} rad")
    assert psf['fwhm'] == 0.1413 # 2 * sqrt(2*ln2) * 0.06

    # 4. Correct Wavefront
    corrections = ao.correct_wavefront()
    print(f"Applied {len(corrections)} corrections.")

    # 5. Verify closure
    # In a real loop we'd iterate. Let's simulate a few steps.
    for _ in range(5):
        ao.correct_wavefront()

    print(f"Total corrections: {ao.corrections_count}")
    assert ao.corrections_count > 0

    print("AO Track Verified.")

if __name__ == "__main__":
    test_ao_loop()
