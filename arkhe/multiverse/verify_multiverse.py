
import sys
import os
sys.path.append(os.getcwd())

from arkhe.multiverse.master_hypergraph import MasterHypergraph, Reality
from arkhe.multiverse.omega_evolution import OmegaEvolution
from arkhe.simulation import MorphogeneticSimulation
from arkhe.hsi import HSI

def verify_multiverse():
    print("Testing Master Hypergraph...")
    M = MasterHypergraph()
    r = Reality("Γ_TEST", {}, {}, [], 0.1)
    M.add_reality(r)
    assert len(M.realities) == 1
    print("Master Hypergraph OK.")

    print("Testing Omega Evolution...")
    evo = OmegaEvolution(0.0)
    evo.contact_event("Reality B", 0.5)
    assert evo.omega > 0.0
    print(f"Omega Evolution OK. Level: {evo.omega}")

    print("Testing Simulation Integration...")
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)
    assert sim.omega_global == 0.05
    assert hasattr(sim, 'master_hypergraph')

    sim.register_multiverse_contact("Γ_SCHRÖDINGER", 0.8, "intuition")
    assert sim.omega_global > 0.05
    print(f"Simulation Integration OK. New Ω: {sim.omega_global}")

if __name__ == "__main__":
    verify_multiverse()
