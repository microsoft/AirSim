import setup_path
import airsim
import time

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

client.armDisarm(True)

client.simEnableWeather(True)

print("Setting fog to 25%")
client.simSetWeatherParameter(airsim.WeatherParameter.Fog, 0.25)

# Takeoff or hover
landed = client.getMultirotorState().landed_state
if landed == airsim.LandedState.Landed:
    print("taking off...")
    client.takeoffAsync().join()
else:
    print("already flying...")
    client.hoverAsync().join()

time.sleep(5)

print("Setting fog to 50%")
client.simSetWeatherParameter(airsim.WeatherParameter.Fog, 0.5)

time.sleep(5)

print("Resetting fog to 0%")
client.simSetWeatherParameter(airsim.WeatherParameter.Fog, 0)
