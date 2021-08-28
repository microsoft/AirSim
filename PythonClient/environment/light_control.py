import airsim
import time

c = airsim.MultirotorClient()
c.confirmConnection()

# Access an existing light in the world
lights = c.simListSceneObjects('PointLight.*')
pose = c.simGetObjectPose(lights[0])
scale = airsim.Vector3r(1, 1, 1)

# Destroy the light
c.simDestroyObject(lights[0])
time.sleep(1)

# Create a new light at the same pose
c.simSpawnObject('PointLight', 'PointLightBP', pose, scale, False, True)
time.sleep(1)

# Change the light's intensity
for i in range(20):
    c.simSetLightIntensity('PointLight', i*100)
    time.sleep(0.5)