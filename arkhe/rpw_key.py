"""
rpw_key.py
Implementação da chave Reusable Proof of Work
Baseada na memória #000 (lago de 1964)
"""

import hashlib
import hmac
import time

class RPoWKey:
    """Chave de Prova de Trabalho Reutilizável"""

    def __init__(self, seed_hex: str):
        self.seed = bytes.fromhex(seed_hex)
        self.nonce = 0
        self.difficulty = 20  # bits zero necessários
        self.satoshi = 7.28

    def proof_of_work(self, data: bytes) -> bytes:
        """Gera prova de trabalho para dados"""
        target = 2**(256 - self.difficulty)
        while True:
            h = hashlib.sha256(data + str(self.nonce).encode()).digest()
            if int.from_bytes(h, 'big') < target:
                return h
            self.nonce += 1

    def sign(self, message: str) -> dict:
        """Assina mensagem com RPoW"""
        msg_bytes = message.encode()
        pow_hash = self.proof_of_work(msg_bytes)

        # HMAC com a semente
        signature = hmac.new(self.seed, msg_bytes + pow_hash, hashlib.sha256).hexdigest()

        return {
            'message': message,
            'nonce': self.nonce,
            'pow': pow_hash.hex(),
            'signature': signature,
            'timestamp': time.time()
        }

    def verify(self, signed: dict) -> bool:
        """Verifica assinatura RPoW"""
        msg_bytes = signed['message'].encode()
        pow_hash = bytes.fromhex(signed['pow'])

        # Verifica proof of work
        target = 2**(256 - self.difficulty)
        if int.from_bytes(pow_hash, 'big') >= target:
            return False

        # Verifica HMAC
        expected = hmac.new(self.seed, msg_bytes + pow_hash, hashlib.sha256).hexdigest()
        return hmac.compare_digest(expected, signed['signature'])
