import time

import numpy as np

class DroneObject(object):
    def __init__(self, droneId):
        self.droneId = droneId
        self.droneName = 'Drone{}'.format(droneId)
        self.currentArena = None
        self.currentStep = 0
        self.droneSpawnOffset = np.array([0, 0 * droneId, 0])

        self.previous_depth_image = None

        self.currentState = None
        self.currentStatePos = None # Used to create the value heat map
        self.previousState = None
        self.currentAction = None
        self.currentTotalReward = 0
        self.distanceFromGoal = None

        self.reseting = True
        self.reseting_API = False
        self.reseting_API_2 = False

        self.resetTick = 0
        self.resetingTime = time.perf_counter()
    def getCurrentArena(self):
        return -1 if self.currentArena == None else self.currentArena