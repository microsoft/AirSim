import setup_path
import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
client.armDisarm(True)
