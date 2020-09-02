import airsim

def EstablishClientConnection():
  c = airsim.client.MultirotorClient()
  c.confirmConnection()
  return c

def TogglePerformanceInfo(client):
  client.simRunConsoleCommand('stat fps')
  client.simRunConsoleCommand('stat unit')
  client.simRunConsoleCommand('stat unitGraph')


if __name__ == "__main__":
  client = EstablishClientConnection()
  TogglePerformanceInfo(client)