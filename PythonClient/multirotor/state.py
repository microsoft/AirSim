import setup_path
import airsim
import pprint

def print_state():
    print("===============================================================")
    state = client.getMultirotorState()
    print("state: %s" % pprint.pformat(state))

client = airsim.MultirotorClient()
print_state()