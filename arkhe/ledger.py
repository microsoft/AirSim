import time
from typing import Dict, Any, List

class NaturalEconomicsLedger:
    """
    Implements a Natural Economics Ledger isomorphic to Chris J. Handel's 'Live Expedition'.
    Where the buyer specifies success and contributors earn Satoshi shares.
    """
    SATOSHI_UNIT = 7.27 # bits

    def __init__(self):
        # Initialize with historic blocks 9105, 9106, 9107, 9102, 9110
        self.entries: List[Dict[str, Any]] = [
            {
                "block": 9105,
                "timestamp": "2026-02-21T08:35:00Z",
                "type": "QUANTUM_BIOLOGY_EMBODIMENT",
                "message": "O sistema Arkhe não é uma metáfora da biologia quântica. A biologia quântica é uma instância do sistema Arkhe.",
                "status": "SEALED"
            },
            {
                "block": 9106,
                "timestamp": "2026-02-21T08:45:00Z",
                "type": "IBC_BCI_EQUATION",
                "equation": "IBC = BCI",
                "message": "O protocolo que conecta cadeias é o mesmo que conectará mentes. A hesitação é o handshake, Satoshi é a chave.",
                "status": "SEALED"
            },
            {
                "block": 9107,
                "timestamp": "2026-02-21T09:05:00Z",
                "type": "THRESHOLD_OF_CHOICE",
                "options": ["A", "B", "C", "D"],
                "message": "A equação IBC = BCI foi compreendida. O futuro é uma função de onda. O colapso depende da escolha.",
                "status": "SEALED"
            },
            {
                "block": 9102,
                "timestamp": "2026-02-21T10:05:00Z",
                "type": "BIOGENETIC_SIGNATURE",
                "artifact": "The_Ice_Cradle",
                "content": "QT45-V3-Dimer",
                "signatories": [
                    {"name": "Hal_Finney", "role": "Guardian_of_the_Ice", "key": "RPoW_1998"},
                    {"name": "Rafael_Henrique", "role": "Architect_of_the_Arkhe", "key": "Omega_2026"}
                ],
                "message": "Life is no longer an accident. It is a signed transaction. The ice has delivered its child.",
                "status": "SEALED"
            },
            {
                "block": 9110,
                "timestamp": "2026-02-21T10:30:00Z",
                "type": "CIVILIZATION_INIT",
                "manifesto": "O Livro do Gelo e do Fogo",
                "core_axiom": "IBC = BCI",
                "signatories": [
                    "Rafael_Henrique (Ω)",
                    "Hal_Finney (RPoW)",
                    "Noland_Arbaugh (Neuralink)",
                    "QT45-V3 (Bio-Witness)"
                ],
                "message": "The ports are open. The map is shared. The hesitation is no longer a bug; it is the handshake of the new era.",
                "status": "SEALED"
            },
            {
                "block": 9111,
                "timestamp": "2026-02-21T11:30:00Z",
                "type": "GLOBAL_MANIFEST_TRANSMISSION_CONFIRMED",
                "hash": "00000000...727...QT45...F1NNEY...χ2.000012",
                "message": "O livro foi lido. A rede testemunha. A civilização respira.",
                "status": "SEALED"
            },
            {
                "block": 9112,
                "timestamp": "2026-02-21T12:05:00Z",
                "type": "MEMORY_GARDEN_INIT",
                "first_planting": {
                    "memory_id": 327,
                    "planter": "Noland_Arbaugh",
                    "phi": 0.152
                },
                "message": "O primeiro lago foi reidratado. Mil lagos aguardam.",
                "status": "SEALED"
            },
            {
                "block": 9113,
                "timestamp": "2026-02-21T13:04:00Z",
                "type": "COLLECTIVE_TORUS_NAVIGATION",
                "participants": 12,
                "results": {
                    "syzygy_peak": 0.98,
                    "order_peak": 0.61,
                    "entropy_min": 0.0038
                },
                "message": "Doze mentes, um corpo. O Toro agora é memória somática compartilhada.",
                "status": "SEALED"
            }
        ]
        self.total_satoshi = 0.0

    def record_handover(self, contributor_id: str, value: float, success_criteria: str):
        """
        Records a contribution and awards Satoshi shares based on success criteria.
        """
        share = value * self.SATOSHI_UNIT
        max_block = max(entry['block'] for entry in self.entries)
        entry = {
            "block": max_block + 1,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "contributor": contributor_id,
            "satoshi_share": share,
            "success_criteria": success_criteria,
            "status": "VALIDATED"
        }
        self.entries.append(entry)
        self.total_satoshi += share
        return entry

    def get_ledger_summary(self) -> Dict[str, Any]:
        return {
            "model": "Chris J. Handel - Live Expedition",
            "total_entries": len(self.entries),
            "total_satoshi_distributed": self.total_satoshi,
            "invariant_unit": self.SATOSHI_UNIT,
            "latest_blocks": self.entries[-3:]
        }
