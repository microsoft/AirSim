import os
import setup_path 
import airsim
import time
import numpy as np
import sys

script_dir = os.path.dirname(__file__)

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

def play_sound(wavfile):
    import speaker
    import wav_reader
    reader = wav_reader.WavReader()
    reader.open(wavfile, 512, speaker.Speaker())
    while True:
        buffer = reader.read()
        if buffer is None:
            break

class Numbers:
    def __init__(self, name):
        self.data = []
        self.name = name

    def add(self, x):
        self.data += [x]

    def is_unstable(self, amount):
        a = np.array(self.data)
        minimum = a.min()
        maximum = a.max()
        mean = np.mean(a)
        stddev = np.std(a)
        print("{}: min={}, max={}, mean={}, stddev={}".format(self.name, minimum, maximum, mean, stddev))
        return (maximum - minimum) > amount

print("### TEST STARTED ###")
print("This test takes 20 minutes.")

iteration = 0
while iteration < 10:
    iteration  += 1
    x = Numbers("x")
    y = Numbers("y")
    z = Numbers("z")
        
    print("arming the drone...")
    client.armDisarm(True)

    while client.getMultirotorState().landed_state == airsim.LandedState.Landed:
        print("taking off...")
        client.takeoffAsync().join()
        time.sleep(1)

    # fly for 2 minutes
    start = time.time()
    while time.time() < start + 120:
        state = client.getMultirotorState()
        x_val = state.kinematics_estimated.position.x_val
        y_val = state.kinematics_estimated.position.y_val
        z_val = state.kinematics_estimated.position.z_val
        x.add(x_val)
        y.add(y_val)
        z.add(z_val)
        print("x: {}, y: {}, z: {}".format(x_val, y_val, z_val))
        time.sleep(1)

    print("landing...")
    client.landAsync().join()
    
    print("disarming the drone...")
    client.armDisarm(False)

    # more than 50 centimeter drift is unacceptable.
    print("Results for iteration {}".format(iteration))
    a = x.is_unstable(0.5)
    b = y.is_unstable(0.5)
    c = z.is_unstable(0.5)

    if a or b or c:
        play_sound(os.path.join(script_dir, "Error.wav"))
        break

    time.sleep(5)

print("### Test Passed ###")
