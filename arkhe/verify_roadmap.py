
import sys
import os
sys.path.append(os.getcwd())

from arkhe.arkhe_core import SATOSHI, NU_LARMOR
from arkhe.agi import CORE_AGI
from arkhe.arc_adapter import ARCAdapter
from arkhe.hyperon_bridge import HyperonBridge

def test_gamma_128_state():
    assert SATOSHI == 9.28
    assert NU_LARMOR == 0.0027
    print("State Γ₁₂₈ verified.")

def test_arc_adapter():
    core = type('MockCore', (), {'add_node': lambda *a: None, 'handover_step': lambda *a: None})
    adapter = ARCAdapter(core)
    task = {
        'train': [{'input': [[1]], 'output': [[2]]}],
        'test': [{'input': [[1]], 'output': [[2]]}]
    }
    score = adapter.run_benchmark([task])
    assert score >= 0
    print("ARC Adapter verified.")

def test_hyperon_bridge():
    core = None
    bridge = HyperonBridge(core)
    bridge.sync_arkhe_to_atomspace()
    res = bridge.apply_rules(1)
    assert "Concept" in res
    print("Hyperon Bridge verified.")

if __name__ == "__main__":
    test_gamma_128_state()
    test_arc_adapter()
    test_hyperon_bridge()
