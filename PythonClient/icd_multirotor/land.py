import setup_path 
import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)



class Land:
    
    def start(self):
        print("land requst") 
        print(client.getMultirotorState())   

        landed = client.getMultirotorState().landed_state
        if landed == airsim.LandedState.Landed:
            print("already landed...")
        else:
            print("landing...")
            client.landAsync().join()

            client.armDisarm(False)
            client.enableApiControl(False)
        return not client.getMultirotorState().collision.has_collided 
