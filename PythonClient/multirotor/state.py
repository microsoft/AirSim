import setup_path
import airsim
import pprint

def print_state():
    print("===============================================================")
    state = client.getMultirotorState()
    print("state: %s" % pprint.pformat(state))
    return state


client = airsim.MultirotorClient()
state = print_state()

if state.ready:
    print("drone is ready!")
else:
    print("drone is not ready!")
if state.ready_message:
    print(state.ready_message)
