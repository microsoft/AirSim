"""
Proof-of-Coherence Authentication
Validators must prove syzygy > threshold to participate
"""

import hashlib
import time
from dataclasses import dataclass
from typing import Optional, List
import json

@dataclass
class CoherenceProof:
    """Cryptographic proof of coherence state"""
    node_id: str
    syzygy: float
    coherence: float
    fluctuation: float
    timestamp: int
    nonce: int
    proof_hash: str

    def verify(self, difficulty: int = 4) -> bool:
        """Verify proof-of-coherence (hash must have leading zeros)"""
        # Recompute hash
        data = f"{self.node_id}{self.syzygy}{self.timestamp}{self.nonce}"
        computed = hashlib.sha256(data.encode()).hexdigest()

        # Check difficulty (leading zeros)
        if not computed.startswith('0' * difficulty):
            return False

        # Check hash matches
        if computed != self.proof_hash:
            return False

        # Verify C+F=1
        if abs(self.coherence + self.fluctuation - 1.0) > 1e-10:
            return False

        return True

class ProofOfCoherenceValidator:
    """
    Generate and verify proof-of-coherence

    Unlike proof-of-work (wasteful), proof-of-coherence requires:
    1. Maintaining high syzygy (useful work)
    2. Mining nonce that encodes coherence state
    """

    def __init__(self, difficulty: int = 4, syzygy_threshold: float = 0.98):
        self.difficulty = difficulty
        self.syzygy_threshold = syzygy_threshold

    def mine_proof(self,
                  node_id: str,
                  syzygy: float,
                  coherence: float = 0.86,
                  fluctuation: float = 0.14,
                  max_iterations: int = 1000000) -> Optional[CoherenceProof]:
        """
        Mine a valid proof-of-coherence

        Args:
            node_id: Unique node identifier
            syzygy: Current syzygy value
            coherence: C value
            fluctuation: F value
            max_iterations: Max mining attempts

        Returns:
            CoherenceProof if found, None otherwise
        """

        # Verify syzygy threshold
        if syzygy < self.syzygy_threshold:
            print(f"Syzygy {syzygy} below threshold {self.syzygy_threshold}")
            return None

        # Verify C+F=1
        if abs(coherence + fluctuation - 1.0) > 1e-10:
            print("C+F≠1 violation")
            return None

        timestamp = int(time.time())
        target = '0' * self.difficulty

        print(f"Mining proof for {node_id} (syzygy={syzygy:.4f})...")

        for nonce in range(max_iterations):
            data = f"{node_id}{syzygy}{timestamp}{nonce}"
            proof_hash = hashlib.sha256(data.encode()).hexdigest()

            if proof_hash.startswith(target):
                proof = CoherenceProof(
                    node_id=node_id,
                    syzygy=syzygy,
                    coherence=coherence,
                    fluctuation=fluctuation,
                    timestamp=timestamp,
                    nonce=nonce,
                    proof_hash=proof_hash
                )

                print(f"✓ Proof found after {nonce+1} iterations")
                print(f"  Hash: {proof_hash}")
                return proof

        print(f"✗ No proof found in {max_iterations} iterations")
        return None

    def verify_proof(self, proof: CoherenceProof) -> bool:
        """Verify a proof-of-coherence"""

        # Check syzygy threshold
        if proof.syzygy < self.syzygy_threshold:
            print(f"Syzygy {proof.syzygy} below threshold")
            return False

        # Verify cryptographic proof
        if not proof.verify(self.difficulty):
            print("Invalid cryptographic proof")
            return False

        return True

    def generate_challenge(self, node_id: str) -> dict:
        """
        Generate a challenge for node to prove coherence

        Returns:
            Challenge dict with timestamp and required difficulty
        """
        return {
            'node_id': node_id,
            'timestamp': int(time.time()),
            'difficulty': self.difficulty,
            'syzygy_threshold': self.syzygy_threshold,
            'expires_at': int(time.time()) + 300  # 5 minutes
        }

class CoherenceAuthenticator:
    """
    Manage authentication tokens based on proof-of-coherence
    """

    def __init__(self):
        self.validator = ProofOfCoherenceValidator(difficulty=4, syzygy_threshold=0.98)
        self.active_tokens: dict[str, str] = {}  # node_id -> token
        self.proofs: dict[str, CoherenceProof] = {}

    def authenticate(self, node_id: str, proof: CoherenceProof) -> Optional[str]:
        """
        Authenticate node and issue token

        Args:
            node_id: Node requesting authentication
            proof: Proof-of-coherence

        Returns:
            Authentication token or None
        """

        if proof.node_id != node_id:
            print("Node ID mismatch")
            return None

        if not self.validator.verify_proof(proof):
            print("Invalid proof")
            return None

        # Generate token
        token_data = f"{node_id}{proof.timestamp}{proof.proof_hash}"
        token = hashlib.sha256(token_data.encode()).hexdigest()

        # Store
        self.active_tokens[node_id] = token
        self.proofs[node_id] = proof

        print(f"✓ Authenticated {node_id}")
        print(f"  Token: {token[:16]}...")

        return token

    def verify_token(self, node_id: str, token: str) -> bool:
        """Verify an authentication token"""

        stored_token = self.active_tokens.get(node_id)
        if not stored_token:
            return False

        return token == stored_token

    def get_node_syzygy(self, node_id: str) -> Optional[float]:
        """Get syzygy of authenticated node"""

        proof = self.proofs.get(node_id)
        if not proof:
            return None

        return proof.syzygy


# Example usage
def example_poc_workflow():
    """Complete proof-of-coherence workflow"""

    print("="*70)
    print("PROOF-OF-COHERENCE AUTHENTICATION EXAMPLE")
    print("="*70)

    # Create authenticator
    auth = CoherenceAuthenticator()

    # Node with high syzygy
    node_id = "arkhe_node_12594"
    syzygy = 0.9836

    print(f"\nNode: {node_id}")
    print(f"Syzygy: {syzygy}")

    # Generate challenge
    challenge = auth.validator.generate_challenge(node_id)
    print(f"\nChallenge issued:")
    print(json.dumps(challenge, indent=2))

    # Node mines proof
    proof = auth.validator.mine_proof(
        node_id=node_id,
        syzygy=syzygy,
        coherence=0.86,
        fluctuation=0.14
    )

    if not proof:
        print("Failed to generate proof")
        return

    print(f"\nProof generated:")
    print(f"  Node: {proof.node_id}")
    print(f"  Syzygy: {proof.syzygy}")
    print(f"  C: {proof.coherence}, F: {proof.fluctuation}")
    print(f"  Nonce: {proof.nonce}")
    print(f"  Hash: {proof.proof_hash}")

    # Authenticate
    token = auth.authenticate(node_id, proof)

    if not token:
        print("Authentication failed")
        return

    print(f"\nAuthentication successful!")
    print(f"Token: {token}")

    # Verify token
    valid = auth.verify_token(node_id, token)
    print(f"\nToken verification: {'✓ VALID' if valid else '✗ INVALID'}")

    # Get syzygy
    node_syzygy = auth.get_node_syzygy(node_id)
    print(f"Retrieved syzygy: {node_syzygy}")

    # Attempt with low syzygy (should fail)
    print("\n" + "="*70)
    print("TESTING LOW SYZYGY (should fail)")
    print("="*70)

    low_syzygy_proof = auth.validator.mine_proof(
        node_id="low_syzygy_node",
        syzygy=0.95,  # Below threshold 0.98
        coherence=0.86,
        fluctuation=0.14
    )

    if low_syzygy_proof:
        print("ERROR: Should have rejected low syzygy")
    else:
        print("✓ Correctly rejected low syzygy node")

if __name__ == "__main__":
    example_poc_workflow()
