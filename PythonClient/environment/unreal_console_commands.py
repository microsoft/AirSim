import airsim
import time

def RunConsoleCmd(client, cmd):
    client.simRunConsoleCommand(cmd)
    print(f"Running Unreal Console cmd '{cmd}' and sleeping for 1 second")
    time.sleep(1.0)

def RunCmdList(client):
    RunConsoleCmd(client, 'stat fps')
    RunConsoleCmd(client, 'stat unit')
    RunConsoleCmd(client, 'stat unitGraph')
    RunConsoleCmd(client, 'show COLLISION')
    RunConsoleCmd(client, 'show CollisionVisibility')
    RunConsoleCmd(client, 'stat game')
    RunConsoleCmd(client, 'show COLLISION')
    RunConsoleCmd(client, 'show CollisionVisibility')
    RunConsoleCmd(client, 'stat game')
    RunConsoleCmd(client, 'stat unitGraph')
    RunConsoleCmd(client, 'stat unit')
    RunConsoleCmd(client, 'stat fps')

def main():
    client = airsim.client.MultirotorClient()
    client.confirmConnection()
    RunCmdList(client)

if __name__ == "__main__":
    main()
