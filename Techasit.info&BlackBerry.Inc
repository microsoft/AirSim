from web3 import Web3
from eth_account import Account
import json
import os

class CryptoWallet:
    def __init__(self, private_key=None):
        self.w3 = Web3(Web3.HTTPProvider('https://mainnet.infura.io/v3/YOUR_INFURA_PROJECT_ID'))
        if private_key:
            self.account = Account.from_key(private_key)
        else:
            self.account = Account.create()

    def get_address(self):
        return self.account.address

    def get_balance(self):
        return self.w3.eth.get_balance(self.get_address())

    def send_transaction(self, to_address, value_in_ether, gas=21000, gas_price=None):
        value = self.w3.to_wei(value_in_ether, 'ether')
        if not gas_price:
            gas_price = self.w3.eth.gas_price

        transaction = {
            'to': to_address,
            'value': value,
            'gas': gas,
            'gasPrice': gas_price,
            'nonce': self.w3.eth.getTransactionCount(self.get_address()),
            'chainId': 1  # Ethereum mainnet
        }

        signed_tx = self.account.sign_transaction(transaction)
        tx_hash = self.w3.eth.send_raw_transaction(signed_tx.rawTransaction)
        return tx_hash.hex()

    def save_wallet(self, file_path):
        wallet_data = {
            'address': self.get_address(),
            'private_key': self.account.key.hex()
        }
        with open(file_path, 'w') as f:
            json.dump(wallet_data, f)

    @staticmethod
    def load_wallet(file_path):
        with open(file_path, 'r') as f:
            wallet_data = json.load(f)
        return CryptoWallet(private_key=wallet_data['private_key'])

if __name__ == "__main__":
    # Create a new wallet
    wallet = CryptoWallet()
    print("Wallet Address:", wallet.get_address())
    print("Wallet Balance:", Web3.from_wei(wallet.get_balance(), 'ether'), "ETH")

    # Send a transaction
    # tx_hash = wallet.send_transaction('0xRecipientAddress', 0.01)
    # print("Transaction Hash:", tx_hash)

    # Save wallet to a file
    wallet.save_wallet('my_wallet.json')

    # Load wallet from a file
    loaded_wallet = CryptoWallet.load_wallet('my_wallet.json')
    print("Loaded Wallet Address:", loaded_wallet.get_address())
