import os
import json
import os
import pathlib
import time

homeDir = str(pathlib.Path.home())
projectName = 'MyProject2'
envProcesses = 2

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
    if i == 0:
        print("Do something here to start drones already.")
    else:
        os.system('gnome-terminal -- ./../UnrealEngineFiles/{projectName}.sh -RenderOffscreen -windowed -NoVSync'.format(projectName=projectName))


        #WinX = 1000 * i
        #WinY = 1000
        #os.system('gnome-terminal -- ./../UnrealEngineFiles/{projectName}.sh -windowed -WinX={WinX} -WinY={WinY} -NoVSync'.format(
        #    projectName=projectName,
        #    WinX=WinX,
        #    WinY=WinY))

    #os.system('gnome-terminal -- nvidia-docker run -it --gpus device={deviceid} -p {port}:{port} --rm -v "$(pwd)/../UnrealEngineFiles/":/UnrealEngineFiles adamrehn/ue4-runtime ./UnrealEngineFiles/MyProject2.sh -RenderOffscreen -windowed -NoVSync'.format(deviceid=i%2, port=port))
    #os.system('gnome-terminal -- nvidia-docker run -it --gpus device={deviceid} -p {port}:{port} --rm -v "$(pwd)/../UnrealEngineFiles/":/UnrealEngineFiles jack/distributedrl:latest ./UnrealEngineFiles/MyProject2.sh -RenderOffscreen -windowed -NoVSync'.format(deviceid=i%2, port=port))

    #jack/distributedrl:latest

    # ip address continues up from 172.17.0.2
    #os.system('gnome-terminal -- ./../UnrealEngineFiles/{projectName}.sh -RenderOffscreen -windowed -NoVSync'.format(projectName=projectName))

    #os.system('gnome-terminal -- ./../UnrealEngineFiles/{projectName}.sh -RenderOffscreen -windowed -NoVSync'.format(projectName=projectName))
    #nvidia-docker run -it --gpus=all --rm -v "$file_location":/ble_lux dev/ble_mount:tensorflow
    #nvidia-docker run -it --gpus=all --rm tensorflow/tensorflow:latest-gpu
    #nvidia-docker run -it --gpus device=1 --rm -v "$file_location":/ble_lux tensorflow/tensorflow:latest-gp
    #tensorflow/tensorflow
    #nvidia-docker run -it --gpus device=1 --rm -v "$(pwd)/../UnrealEngineFiles/":/UnrealEngineFiles tensorflow/tensorflow:latest-gpu
    #nvidia-docker run -it --gpus device=1 --rm -v "$(pwd)/../UnrealEngineFiles/":/UnrealEngineFiles adamrehn/ue4-runtime ./UnrealEngineFiles/MyProject2.sh -RenderOffscreen -windowed -NoVSync

    #

    #useradd -ms /bin/bash newuser
    #su - newuser
    #./../../UnrealEngineFiles/MyProject2.sh -RenderOffscreen -windowed -NoVSync
    time.sleep(4)

time.sleep(5)
for i in range(envProcesses):
    UE_port = storage_port + i + 1
    if i == 0:
        os.system('gnome-terminal -- python PyClient.py {UE_port} {UE_Address} {storage_port}'.format(UE_port=29004, UE_Address="192.168.1.110", storage_port=storage_port))
        print(UE_port)
    else:
        os.system('gnome-terminal -- python PyClient.py {UE_port} {UE_Address} {storage_port}'.format(UE_port=UE_port, UE_Address="127.0.0.1", storage_port=storage_port))