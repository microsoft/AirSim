
import sys
import os
sys.path.append(os.getcwd())

from arkhe.ibc_bci import IBC_BCI
from arkhe.pineal import PinealTransducer

def test_ibc_bci():
    protocol = IBC_BCI(phi=0.15)
    packet = {"type": "intent", "value": "awaken"}
    relayed = protocol.relay_packet(packet)
    assert relayed["status"] == "confirmed"
    assert relayed["proof"] == 0.94
    print("IBC=BCI test passed.")

def test_pineal():
    pineal = PinealTransducer()
    # Test piezoelectricity
    voltage = pineal.calculate_voltage(phi=0.15)
    assert round(voltage, 2) == 0.94

    # Test RPM
    state = pineal.radical_pair_mechanism(phi=0.15)
    assert state == "Singlet"
    print("Pineal test passed.")

if __name__ == "__main__":
    test_ibc_bci()
    test_pineal()
