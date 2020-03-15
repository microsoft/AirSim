import setup_path 
import airsim





class Land:
    
    def __init__(self, port):
        self.port = port
      
    
    def start(self):
        client = airsim.MultirotorClient('',self.port)
        client.confirmConnection()
        client.enableApiControl(True)
        client.armDisarm(True)
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
