from arkhe.calcium import CalciumEngine
import numpy as np

def test_proprioception_track():
    print("--- Verifying Proprioceptive Rest Track (Γ_∞+49) ---")
    calc = CalciumEngine()

    # 1. Position check
    pos = calc.drone_position
    print(f"Current Position: {pos}")
    assert np.allclose(pos, [50.0, 0.0, -9.99])

    # 2. Muscle tone check
    print(f"Muscle Tone: {calc.muscle_tone} rad")
    assert calc.muscle_tone == 0.7353

    # 3. Proprioception report
    report = calc.get_proprioception_report()
    print(f"Proprioceptive Status: {report['status']}")
    assert report['status'] == "HOVER_CONSCIENTE"
    assert report['prediction_error'] == 0.0
    assert "É presença" in report['message']

    # 4. Drone status check
    status = calc.get_drone_status()
    print(f"Satoshi context: {status['satoshi']} bits")
    assert status['satoshi'] == 7.26999862

    print("Proprioception Track Verified.")

if __name__ == "__main__":
    test_proprioception_track()
