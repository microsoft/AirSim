
import sys
import os
sys.path.append(os.getcwd())

from arkhe.ibc_bci import IBCBCI
from arkhe.pineal import PinealTransducer
from arkhe.alpha_omega import AlphaOmega
from arkhe.eigen import ArkheEigen
from arkhe.pfas import PFAS_ReMADE
from arkhe.reversion import CancerReversion
from arkhe.agi import AGIEngine

def test_ibc_bci():
    protocol = IBCBCI()
    relayed = protocol.relay_hesitation(source_omega=0.07, target_omega=0.00, hesitation=0.15)
    assert relayed["satoshi"] == 8.72
    print("IBC=BCI test passed.")

def test_pineal():
    pineal = PinealTransducer()
    assert pineal.SATOSHI == 8.72
    print("Pineal test passed.")

def test_reversion():
    reversion = CancerReversion()
    res = reversion.simulate_reversion(0.9)
    assert res["status"] == "REVERSION_SUCCESS"
    print("Cancer Reversion test passed.")

def test_agi():
    agi = AGIEngine()
    val = agi.calculate_potential()
    assert abs(val) > 0
    print("AGI Formula test passed.")

if __name__ == "__main__":
    test_ibc_bci()
    test_pineal()
    test_reversion()
    test_agi()
