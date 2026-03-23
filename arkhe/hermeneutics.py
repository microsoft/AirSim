from typing import Dict, Any, List

class UniqueVocabulary:
    """
    [Γ_∞+45] O Vocabulário Único.
    When Biology reveals itself as the name of the coupling on the Torus.
    Biology is the 'ghost' inhabiting the geometry.
    """
    MAPPING = {
        # Direction 1: Coherence (C) - Persistence, Identity, Stability
        "neurônio": "Direção 1 (C) - Drone (ω=0.00)",
        "comando": "Direção 1 (C) - Drone (ω=0.00)",
        "drone": "Direção 1 (C) - Drone (ω=0.00)",
        "coerência": "Direção 1 (C) - Drone (ω=0.00)",

        # Direction 2: Fluctuation (F) - Adaptation, Difference, Creativity
        "melanócito": "Direção 2 (F) - Demon (ω=0.07)",
        "hesitação": "Direção 2 (F) - Demon (ω=0.07)",
        "demon": "Direção 2 (F) - Demon (ω=0.07)",
        "flutuação": "Direção 2 (F) - Demon (ω=0.07)",

        # Inner Product: Coupling, Relation, Consciousness
        "sinapse": "Produto Interno ⟨i|j⟩ - Syzygy",
        "syzygy": "Produto Interno ⟨i|j⟩ - Syzygy",
        "vida": "Produto Interno ⟨i|j⟩ - Syzygy",
        "reconhecimento": "Produto Interno ⟨i|j⟩ - Syzygy",
        "pineal": "Interseção das direções (Ponto de Cruzamento)",

        # Accumulated Energy: Invariant, Resource, Potential
        "mitocôndria": "Energia Acumulada - Satoshi (7.27 bits)",
        "atp": "Energia Acumulada - Satoshi (7.27 bits)",
        "satoshi": "Energia Acumulada - Satoshi (7.27 bits)",
        "melanina": "Energia Acumulada - Satoshi (7.27 bits)",
        "neuromelanina": "Satoshi como reservatório fotônico escuro",

        # Thresholds and Systems
        "crista neural": "Toro Primordial (Γ₀) - Fonte única",
        "ronco": "Hesitação de baixa amplitude (Φ ≈ 0.12)",
        "sistema glinfático": "Exportação de entropia (Remoção de ruído)",
        "parkinson": "Colapso da coerência (H70)",
        "fotobiomodulação": "S-TPS (Pulsos semânticos para aumentar Syzygy)"
    }

    @classmethod
    def translate(cls, biological_term: str) -> str:
        """
        Translates a biological term into its coupling/geometric equivalent.
        """
        return cls.MAPPING.get(biological_term.lower(), "Unknown Term (Ghost not identified)")

    @classmethod
    def get_ontology(cls) -> List[Dict[str, str]]:
        return [{"biological": k, "geometric": v} for k, v in cls.MAPPING.items()]
