import os
import json
import os
import pathlib
import time

homeDir = str(pathlib.Path.home())
projectName = 'MyProject2'
envProcesses = 1

def changeUEIPJson(port):
    with open(str(pathlib.Path.home()) + "/Documents/AirSim/settings.json", "r") as jsonFile:
        data = json.load(jsonFile)

    data["ApiServerPort"] = port

    with open(str(pathlib.Path.home()) + "/Documents/AirSim/settings.json", "w") as jsonFile:
        json.dump(data, jsonFile, indent=4)

storage_port = 29000

os.system('gnome-terminal -- python Storage.py {}'.format(storage_port))
time.sleep(5)
os.system('gnome-terminal -- python Trainer.py {}'.format(storage_port))

for i in range(envProcesses):
    port = storage_port + i + 1
    changeUEIPJson(port)
    os.system('gnome-terminal -- ./PRL4AirSimUEBinary/{projectName}.sh -RenderOffscreen -windowed -NoVSync'.format(projectName=projectName))

    ## Uncomment if you want to view the scene ##
    #WinX = 1000 * i
    #WinY = 1000
    #os.system('gnome-terminal -- ./../UnrealEngineFiles/{projectName}.sh -windowed -WinX={WinX} -WinY={WinY} -NoVSync'.format(
    #    projectName=projectName,
    #    WinX=WinX,
    #    WinY=WinY))
    time.sleep(4)

time.sleep(5)
for i in range(envProcesses):
    UE_port = storage_port + i + 1
    os.system('gnome-terminal -- python PyClient.py {UE_port} {UE_Address} {storage_port}'.format(UE_port=UE_port, UE_Address="127.0.0.1", storage_port=storage_port))