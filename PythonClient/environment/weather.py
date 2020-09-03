import setup_path 
import airsim


client = airsim.VehicleClient()
client.confirmConnection()

client.simEnableWeather(True)

airsim.wait_key('Press any key to enable rain at 25%')
client.simSetWeatherParameter(airsim.WeatherParameter.Rain, 0.25);

airsim.wait_key('Press any key to enable rain at 75%')
client.simSetWeatherParameter(airsim.WeatherParameter.Rain, 0.75);

airsim.wait_key('Press any key to enable snow at 50%')
client.simSetWeatherParameter(airsim.WeatherParameter.Snow, 0.50);

airsim.wait_key('Press any key to enable maple leaves at 50%')
client.simSetWeatherParameter(airsim.WeatherParameter.MapleLeaf, 0.50);

airsim.wait_key('Press any key to set all effects to 0%')
client.simSetWeatherParameter(airsim.WeatherParameter.Rain, 0.0);
client.simSetWeatherParameter(airsim.WeatherParameter.Snow, 0.0);
client.simSetWeatherParameter(airsim.WeatherParameter.MapleLeaf, 0.0);

airsim.wait_key('Press any key to enable dust at 50%')
client.simSetWeatherParameter(airsim.WeatherParameter.Dust, 0.50);

airsim.wait_key('Press any key to enable fog at 50%')
client.simSetWeatherParameter(airsim.WeatherParameter.Fog, 0.50);

airsim.wait_key('Press any key to disable all weather effects')
client.simEnableWeather(False)
