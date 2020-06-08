import setup_path
import airsim

client = airsim.MultirotorClient()
client.armDisarm(False)
