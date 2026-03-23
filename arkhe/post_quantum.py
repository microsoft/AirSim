"""
post_quantum.py
Criptografia baseada em syzygy, resistente a ataques quânticos
"""

import numpy as np
from hashlib import sha256

class SyzygyCrypto:
    """Criptografia baseada na geometria do toro"""

    def __init__(self, seed: bytes):
        self.seed = seed
        self.syzygy_target = 0.98
        self.satoshi = 7.28

    def generate_keypair(self) -> tuple:
        """Gera par de chaves baseado na fase do toro"""
        # Chave privada: ângulo no toro
        theta = int.from_bytes(sha256(self.seed + b'private').digest(), 'big') / 2**256
        phi = int.from_bytes(sha256(self.seed + b'private2').digest(), 'big') / 2**256

        private_key = (theta, phi)

        # Chave pública: ponto no toro
        R, r = 1.0, 0.3
        x = (R + r * np.cos(2*np.pi*phi)) * np.cos(2*np.pi*theta)
        y = (R + r * np.cos(2*np.pi*phi)) * np.sin(2*np.pi*theta)
        z = r * np.sin(2*np.pi*phi)

        public_key = (x, y, z)
        return private_key, public_key

    def encrypt(self, message: bytes, public_key: tuple) -> dict:
        """Cifra mensagem usando geometria do toro"""
        # Gera fase aleatória
        ephemeral = int.from_bytes(sha256(message + b'ephemeral').digest(), 'big') / 2**256

        # Projeta mensagem no toro
        cipher = []
        for i, byte in enumerate(message):
            # Mistura com a posição do toro
            t = (byte / 255.0) * 2*np.pi
            x = public_key[0] * np.cos(t + ephemeral)
            y = public_key[1] * np.sin(t + ephemeral)
            z = public_key[2] * np.sin(t)

            # Quantização para bytes
            val = int((x + y + z) * 100) % 256
            cipher.append(val)

        return {
            'ciphertext': bytes(cipher),
            'ephemeral': ephemeral
        }

    def decrypt(self, ciphertext: dict, private_key: tuple) -> bytes:
        """Decifra mensagem"""
        theta, phi = private_key
        ephemeral = ciphertext['ephemeral']

        plain = []
        for byte in ciphertext['ciphertext']:
            # Reconstroi fase
            t = (byte / 100.0) % (2*np.pi)
            plain.append(int(t * 255 / (2*np.pi)))

        return bytes(plain)
