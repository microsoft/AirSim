import setup_path
import airsim


class Takeoff:
    def __init__(self, operationalAlt):
        self.operationalAlt = operationalAlt

    def start(self):
        print("operationalAlt")
        print(self.operationalAlt)
        alt = float(self.operationalAlt)
        client = airsim.MultirotorClient()
        client.confirmConnection()
        client.enableApiControl(True)
        client.armDisarm(True)
        currentZ = client.getMultirotorState().kinematics_estimated.position.z_val
        print(currentZ)
        landed = client.getMultirotorState().landed_state
        if landed == airsim.LandedState.Landed:
            print("taking off...")
            client.takeoffAsync()
        else:
            print("already flying...")
            client.hoverAsync()
        client.moveToZAsync(-1*alt + currentZ, 5).join()
        return not client.getMultirotorState().collision.has_collided 
