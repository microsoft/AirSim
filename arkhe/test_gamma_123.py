
import sys
import os
sys.path.append(os.getcwd())

from arkhe.arkhe_core import SATOSHI
from arkhe.agi import CORE_AGI, AGIEngine
from arkhe.pineal import PinealTransducer
from arkhe.ibc_bci import IBCBCI

def test_gamma_123_telemetry():
    assert SATOSHI == 8.89
    print("Telemetry Γ₁₂₃ verified.")

def test_agi_formula():
    engine = AGIEngine()
    potential = engine.calculate_potential()
    magnitude = abs(potential)
    assert magnitude > 8.0 # Satoshi base
    print(f"AGI Potential Magnitude: {magnitude:.4f}")
    print("AGI Formula verified.")

def test_core_architecture():
    agi = CORE_AGI()
    result = agi.run("Test Command")
    assert result["feedback"] is True
    print("CORE Architecture verified.")

def test_pineal_embodiment():
    pineal = PinealTransducer()
    voltage = pineal.calculate_piezoelectricity(0.15)
    assert round(voltage, 2) == 0.94 # 6.27 * 0.15
    print("Pineal Embodiment verified.")

if __name__ == "__main__":
    test_gamma_123_telemetry()
    test_agi_formula()
    test_core_architecture()
    test_pineal_embodiment()
