import hashlib
import json
import time
from typing import List, Dict, Any

class BlockchainLedger:
    """
    Γ₁₃₀: Camada de Auditabilidade (ARKHE-GATE).
    Registra handovers em uma estrutura de blockchain imutável.
    """
    def __init__(self):
        self.chain: List[Dict[str, Any]] = []
        self.validators = ["MIT", "CERN", "ETH Zurich", "USP", "Max Planck", "Oxford", "Tokyo U"]
        self._create_genesis()

    def _create_genesis(self):
        self.add_block(handover_count=0, block_type="genesis", data={"message": "Arkhe Genesis"})

    def add_block(self, handover_count: int, block_type: str, data: Dict[str, Any]):
        block = {
            "block_id": len(self.chain) + 1048576,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "handover_count": handover_count,
            "type": block_type,
            "data": data,
            "previous_hash": self.chain[-1]["hash"] if self.chain else "0",
        }
        block["hash"] = self._hash_block(block)
        self.chain.append(block)
        return block["block_id"]

    def _hash_block(self, block: Dict[str, Any]) -> str:
        block_string = json.dumps(block, sort_keys=True).encode()
        return hashlib.sha256(block_string).hexdigest()

    def verify_chain(self) -> bool:
        for i in range(1, len(self.chain)):
            if self.chain[i]["previous_hash"] != self.chain[i-1]["hash"]:
                return False
        return True

ledger_instance = BlockchainLedger()

def register_handover(handover_count: int, block_type: str, data: Dict[str, Any]):
    return ledger_instance.add_block(handover_count, block_type, data)
