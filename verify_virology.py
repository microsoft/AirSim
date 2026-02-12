from arkhe.hsi import HSI
from arkhe.simulation import MorphogeneticSimulation, Focus, MonolayerStatus, FocusFate

def test_virology_titration():
    print("Testing FFU Titration...")
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # Formula: foci_count * (1.0 / dilution) * (1.0 / volume)
    # 1 * (1/0.1) * (1/1) = 10
    titer = sim.titular_oncogene(foci_count=1, dilution=0.1, volume=1.0)
    assert titer == 10.0

    # WP1: 10^1 = 10
    # DVM-1: 10^2 = 100
    # Bola: 10^3 = 1000
    assert sim.titular_oncogene(1, 0.1, 1.0) == 10.0
    assert sim.titular_oncogene(1, 0.01, 1.0) == 100.0
    assert sim.titular_oncogene(1, 0.001, 1.0) == 1000.0

    print("Titration verified.")

def test_focus_fate_and_replication():
    print("Testing Focus Fate and Replication...")
    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # 1. Virgin state -> Latent
    sim.monolayer_status = MonolayerStatus.VIRGIN
    focus_v = sim.induzir_turbulencia()
    assert focus_v.fate == FocusFate.LATENT

    # 2. Restored state -> Lytic
    sim.darvo_abort(reason="test")
    assert sim.monolayer_status == MonolayerStatus.RESTORED

    # Induction in restored state
    focus_r = sim.induzir_turbulencia()
    assert focus_r.fate == FocusFate.LYTIC

    # 3. Replication of a Pedra (WP1) in Restored state
    # Setup WP1 as a Pedra
    sim.foci["WP1"] = Focus(name="WP1", integrity=0.97, fate=FocusFate.LATENT, autonomous=True)

    replica = sim.replicar_foco("WP1", (0,0,0,0))
    assert replica.fate == FocusFate.LYTIC # Fate is determined by current status (Restored)
    assert replica.autonomous == False

    print("Fate and replication verified.")

if __name__ == "__main__":
    try:
        test_virology_titration()
        test_focus_fate_and_replication()
        print("\n✅ ALL VIROLOGICAL VERIFICATIONS PASSED.")
    except Exception as e:
        print(f"\n❌ VERIFICATION FAILED: {e}")
        import traceback
        traceback.print_exc()
        exit(1)
