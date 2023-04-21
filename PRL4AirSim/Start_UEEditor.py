import os
import json
import os
import pathlib
import time
import Utils

UEEditor_port = 29001
storage_port = 29000

def changeUEIPJson(port):
    with open(str(pathlib.Path.home()) + "/Documents/AirSim/settings.json", "r") as jsonFile:
        data = json.load(jsonFile)

    data["ApiServerPort"] = port

    with open(str(pathlib.Path.home()) + "/Documents/AirSim/settings.json", "w") as jsonFile:
        json.dump(data, jsonFile, indent=4)

changeUEIPJson(UEEditor_port)


os.system('gnome-terminal -- python Storage.py {}'.format(storage_port))
time.sleep(10)
os.system('gnome-terminal -- python PyClient.py {UE_port} {UE_Address} {storage_port}'.format(UE_port=UEEditor_port, UE_Address="127.0.0.1", storage_port=storage_port))
os.system('gnome-terminal -- python Trainer.py {}'.format(storage_port))