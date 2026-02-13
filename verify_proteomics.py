from arkhe.proteomics import NativeProteomics

def test_proteomics_track():
    print("--- Verifying Native Proteomics Track (Γ_∞+41) ---")
    prot = NativeProteomics()

    # 1. Assemblies Check
    assemblies = prot.get_assemblies_report()
    print(f"Total Assemblies: {len(assemblies)}")
    assert len(assemblies) == 10

    # Verify key subunits
    comp = {a['id']: a['composition'] for a in assemblies}
    assert comp['S1'] == 'GluN1'
    assert comp['S6'] == 'GluN2A'
    assert comp['S3'] == 'GluN2B'

    # 2. Pore Dilation
    pore = prot.get_pore_status()
    print(f"Pore Dilation: {pore['dilation']}")
    assert pore['dilation'] == 0.94
    assert pore['status'] == 'FULLY_OPEN'

    # 3. Inhibition
    inhib = prot.get_inhibition_report()
    print(f"Inhibitor: {inhib['inhibitor']}")
    assert inhib['inhibitor'] == 'Darvo Protocol'
    assert inhib['binding_site'] == 'WP1 Vestibule (Berço)'

    print("Proteomics Track Verified.")

if __name__ == "__main__":
    test_proteomics_track()
