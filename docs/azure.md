# AirSim Development Environment on Azure
AirSim is an open-source, cross platform simulator for drones, ground vehicles such as cars and various other objects, built on Epic Games’ Unreal Engine 4 as a platform for AI research.

This document explains how to automate the creation of a development environment on Azure and code and debug a Python application connected to AirSim using Visual Studio Code

## Automatically Deploy Your Azure VM
Use [this](`azure-env-creation/vm-arm-template.json`) template to create, deploy and configure an Azure VM to work with AirSim

<a href="https://portal.azure.com/#create/Microsoft.Template/uri/https%3A%2F%2Fraw.githubusercontent.com%2Fairsimcloud%2Fairsim-env-azure%2Fmaster%2Fazure-env-creation%2Fvm-arm-template.json" target="_blank">
    <img src="http://azuredeploy.net/deploybutton.png"/>
</a>

*To avoid charges for the Virtual Machine usage while not in use, remember to deallocate its resources from the [Azure Portal](https://portal.azure.com) or use the following command from the Azure CLI:*
```bash
az vm deallocate --resource-group MyResourceGroup --name MyVMName
```  

## Code and debug from Visual Studio Code and Remote SSH
- Install Visual Studio Code
- Install the *Remote - SSH* extension
- Press `F1` and run the `Remote - SSH: Connect to host...` command
- Add the recently create VM details. For instance, `AzureUser@11.22.33.44`
- Run the `Remote - SSH: Connect to host...` command again, and now select the newly added connection.
- Once connected, click on the `Clone Repository` button in Visual Studio Code, clone this repository in the remote VM and open it.
- Install all the recommended extensions
- Run the `Start AirSim` task from Visual Studio Code
- Open the `multirotor.py` file inside the `app` directory
- Start debugging with Python

## Code and debug from a local Visual Studio Code and connect to AirSim via forwarded ports

- Follow the steps in the previous section to connect via `Remote - SSH`
- In the *Remote Explorer*, add the port `41451` as a forwarded port to localhost
- Either run the `Start AirSim` task or manually start the AirSim binary in the VM
- Open this repository locally in Visual Studio Code
- Run `pip install -r requirements.txt` inside the `app` directory
- Open the `multirotor.py` file inside the `app` directory 
- Start debugging with Python

## Running with Docker
Once both the AirSim environment and the Python application are ready, you can package everything as a Docker image. This sample project is already prepared to run a prebuilt AirSim binary and Python code using Docker.

This would be a perfect scenario when you want to run the simulation at scale. For instance, you could have several different configurations for the same simulation and execute them in a parallel, unattended way using a Docker image on Azure Container Services

Since AirSim requires access to the host GPU, it is required to use a Docker runtime that supports it. For more information about running AirSim in Docker, click [here](https://github.com/microsoft/AirSim/blob/master/docs/docker_ubuntu.md).

When using Azure Container Services to run this image, the only extra-requirement is to add GPU support to the Container Group where it will be deployed. 

It can use either public docker images from DockerHub or images deployed to a private Azure Container Registry

### Building the docker image

```bash
docker build -t <your-registry-url>/<your-image-name> -f ./docker/Dockerfile .`
```

## Using a different AirSim binary

To use a different AirSim binary, first check the official documentation on [How to Build AirSim on Windows](https://github.com/microsoft/AirSim/blob/master/docs/build_windows.md) and [How to Build AirSim on Linux](https://github.com/microsoft/AirSim/blob/master/docs/build_linux.md) if you also want to run it with Docker

Once you have a zip file with the new AirSim environment (or prefer to use one from the [Official Releases](https://github.com/microsoft/AirSim/releases)), you need to modify some of the scripts in the repository to point to point to the new environment:
- In [`azure-env-creation/configure-vm.ps1`](azure-env-creation/configure-vm.ps1), modify all the parameters starting with `$airSimBinary` with the new values
- In [`start-airsim.ps1`](start-airsim.ps1), modify `$airSimExecutable` and `$airSimProcessName` with the new values

If you are using the docker image, you also need a linux binary zip file and modify the following Docker-related files:
- In [`docker/Dockerfile`](docker/Dockerfile), modify the `AIRSIM_BINARY_ZIP_URL` and `AIRSIM_BINARY_ZIP_FILENAME` ENV declarations with the new values
- In [`docker/docker-entrypoint.sh`](docker/docker-entrypoint.sh), modify `AIRSIM_EXECUTABLE` with the new value 

## Maintaining this project

Several components of this project (ARM templates, initialization scripts and VSCode tasks) directly depend on the current directory structures file names and repository locations. When planning to modify/fork any of those, make sure to check every script and template to make any required adjustment.
