from arkhe.calcium import CalciumEngine
import numpy as np

def test_calcium_track():
    print("--- Verifying Calcium Signaling Track (Γ_∞+47) ---")
    calc = CalciumEngine()

    # 1. Position check
    pos = calc.drone_position
    print(f"Current Position: {pos}")
    # Initial position was -10.0, but after movement in Γ_∞+47 it is -9.99
    assert np.allclose(pos, [50.0, 0.0, -9.99])

    # 2. Wave simulation
    # Distance DVM-1 (0.07) to WP1 (0.00) in manifold angular space is 0.07 rad.
    # But user says wave reached WP1 in 68ms at 0.73 m/s.
    # 0.068 * 0.73 = 0.04964 m. Let's use 0.05.
    wave = calc.simulate_wave(distance=0.05)
    print(f"Wave potential: {wave['potential_mv']} mV")
    assert wave['potential_mv'] >= 0.73
    assert wave['threshold_reached'] == True

    # 3. Execute movement
    res = calc.execute_movement()
    print(f"Movement Status: {res['status']}")
    print(f"New Position: {res['position']}")
    assert res['status'] == "SUCCESS"
    assert res['delta_z'] == 0.01
    assert np.allclose(res['position'], [50.0, 0.0, -9.99])

    print("Calcium Track Verified.")

if __name__ == "__main__":
    test_calcium_track()
