# AirSim on Docker in Linux

## Binaries
#### Requirements:
- Install [nvidia-docker2](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-2.0))

#### Build the docker image
- Below are the default arguments.   
  `--base_image`: This is image over which we'll install airsim. We've tested on Ubuntu 16.04 + CUDA 10.0.  
   You can specify any [NVIDIA cudagl](https://hub.docker.com/r/nvidia/cudagl/) at your own risk.    
   `--target_image` is the desired name of your docker image.    
   Defaults to `airsim_binary` with same tag as the base image
   ```
   $ cd Airsim/docker;
   $ python build_airsim_image.py \
      --base_image=nvidia/cudagl:10.0-devel-ubuntu16.04 \
      --target_image=airsim_binary:10.0-devel-ubuntu16.04
   ```

- Verify you have an image by:
 `$ docker images | grep airsim`   

#### Running an unreal binary inside a docker container 
- Get [an unreal binary](https://github.com/Microsoft/AirSim/releases/tag/v1.2.0Linux) or package your own project in Ubuntu.   
Let's take the Blocks binary as an example.   
You can download it by running
    ```
    $ cd Airsim/docker;
    $ ./download_blocks_env_binary.sh
    ```

- Running an unreal binary inside a docker container 
   The syntax is:
   ```
    $ ./run_airsim_image_binary.sh DOCKER_IMAGE_NAME UNREAL_BINARY_SHELL_SCRIPT UNREAL_BINARY_ARGUMENTS -- headless     
   ```
   For blocks, you can do a `$ ./run_airsim_image_binary.sh airsim_binary:10.0-devel-ubuntu16.04 Blocks/Blocks.sh -windowed -ResX=1080 -ResY=720`

   * `DOCKER_IMAGE_NAME`: Same as `target_image` parameter in previous step. By default, enter `airsim_binary:10.0-devel-ubuntu16.04`   
   * `UNREAL_BINARY_SHELL_SCRIPT`: for Blocks enviroment, it will be `Blocks/Blocks.sh`
   * [`UNREAL_BINARY_ARGUMENTS`](https://docs.unrealengine.com/en-us/Programming/Basics/CommandLineArguments):
      For airsim, most relevant would be `-windowed`, `-ResX`, `-ResY`. Click on link to see all options. 
         
    * Running in Headless mode:    
        Suffix `-- headless` at the end:
        ```
        $ ./run_airsim_image_binary.sh Blocks/Blocks.sh -- headless
        ```
- [Specifying a `settings.json`](https://github.com/madratman/AirSim/blob/PR/docker_ubuntu/docs/docker_ubuntu.md#airsim_binary-docker-image)

## Source
#### Requirements:
- Install [nvidia-docker2](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-2.0))
- Install [ue4-docker](https://adamrehn.com/docs/ue4-docker/configuration/configuring-linux)

#### Build Unreal Engine inside docker:
 * To get access to Unreal Engine's source code, register on Epic Games' website and link it to your github account, as explained in the `Required Steps` section [here](https://docs.unrealengine.com/en-us/Platforms/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow).    
    Note that you don't need to do `Step 2: Downloading UE4 on Linux`! 

 * Build unreal engine 4.19.2 docker image. We're going to use CUDA 10.0 in our example.    
    `$ ue4-docker build 4.19.2 --cuda=10.0 --no-full`   
    [optional] `$ ue4-docker clean` to free up some space. [Details here](https://adamrehn.com/docs/ue4-docker/commands/clean) 
   - `ue4-docker` supports all CUDA version listed on NVIDIA's cudagl dockerhub [here](https://hub.docker.com/r/nvidia/cudagl/).    
   - Please see [this page](https://adamrehn.com/docs/ue4-docker/building-images/advanced-build-options) for advanced configurations using `ue4-docker`   

 * Disk space:
   - The unreal images and containers can take up a lot of space, especially if you try more than one version.    
   - Here's a list of useful links to monitor space used by docker and clean up intermediate builds:
     * [Large container images primer](https://adamrehn.com/docs/ue4-docker/read-these-first/large-container-images-primer)  
     * [$ `docker system df`](https://docs.docker.com/engine/reference/commandline/system_df/)   
       [$ `docker container prune`](https://docs.docker.com/engine/reference/commandline/container_prune/)   
       [$ `docker image prune`](https://docs.docker.com/engine/reference/commandline/image_prune/)   
       [$ `docker system prune`](https://docs.docker.com/engine/reference/commandline/system_df/)   

#### Building AirSim inside UE4 docker container:
* Build AirSim docker image (which lays over the unreal image we just built)   
  Below are the default arguments.   
  `--base_image`: This is image over which we'll install airsim. We've tested on `adamrehn/ue4-engine:4.19.2-cudagl10.0`. See [ue4-docker](https://adamrehn.com/docs/ue4-docker/building-images/available-container-images) for other versions.     
   `--target_image` is the desired name of your docker image.    
   Defaults to `airsim_source` with same tag as the base image
   ```
   $ cd Airsim/docker;
   $ python build_airsim_image.py \
      --source \
      ----base_image adamrehn/ue4-engine:4.19.2-cudagl10.0 \
      --target_image=airsim_source:4.19.2-cudagl10.0
   ```
#### Running AirSim container
* Run the airsim source image we built by:
   ```
      ./run_airsim_image_source.sh airsim_source:4.19.2-cudagl10.0
   ```
   Syntax is `./run_airsim_image_source.sh DOCKER_IMAGE_NAME -- headless`
   `-- headless`: suffix this to run in optional headless mode. 

* Inside the container, you can see `UnrealEngine` and `AirSim` under `/home/ue4`. 
* Start unreal engine inside the container:   
   `ue4@HOSTMACHINE:~$ /home/ue4/UnrealEngine/Engine/Binaries/Linux/UE4Editor`
* [Specifying an airsim settings.json](https://github.com/madratman/AirSim/blob/PR/docker_ubuntu/docs/docker_ubuntu.md#airsim_source-docker-image)
* Continue with [AirSim's Linux docs](https://microsoft.github.io/AirSim/docs/build_linux/#build-unreal-environment). 

#### [Misc] Packaging Unreal Environments in `airsim_source` containers
* Let's take the Blocks environment as an example.    
    In the following script, specify the full path to your unreal uproject file by `project` and the directory where you want the binaries to be placed by `archivedirectory` 
    ```
    $ /home/ue4/UnrealEngine/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun -platform=Linux -clientconfig=Shipping -serverconfig=Shipping -noP4 -cook -allmaps -build -stage -prereqs -pak -archive \
    -archivedirectory=/home/ue4/Binaries/Blocks/ \
    -project=/home/ue4/AirSim/Unreal/Environments/Blocks/Blocks.uproject
    ```

    This would create a Blocks binary in `/home/ue4/Binaries/Blocks/`.   
    You can test it by running `/home/ue4/Binaries/Blocks/LinuxNoEditor/Blocks.sh -windowed`   

### Specifying an airsim settings.json
  #### `airsim_binary` docker image:
  - We're mapping the host machine's `PATH/TO/Airsim/docker/settings.json` to the docker container's `/home/airsim_user/Documents/AirSim/settings.json`.    
  - Hence, we can load any settings file by simply modifying `PATH_TO_YOUR/settings.json` by modifying the following snippets in * [`run_airsim_image_binary.sh`](https://github.com/Microsoft/AirSim/blob/master/docker/run_airsim_image_binary.sh)
      ```
      nvidia-docker run -it \
          -v $PATH_TO_YOUR/settings.json:/home/airsim_user/Documents/AirSim/settings.json \
          -v $UNREAL_BINARY_PATH:$UNREAL_BINARY_PATH \
          -e SDL_VIDEODRIVER=$SDL_VIDEODRIVER_VALUE \
          -e SDL_HINT_CUDA_DEVICE='0' \
          --net=host \
          --env="DISPLAY=$DISPLAY" \
          --env="QT_X11_NO_MITSHM=1" \
          --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
          -env="XAUTHORITY=$XAUTH" \
          --volume="$XAUTH:$XAUTH" \
          --runtime=nvidia \
          --rm \
          $DOCKER_IMAGE_NAME \
          /bin/bash -c "$UNREAL_BINARY_COMMAND"
      ```
  ####  `airsim_source` docker image:

  * We're mapping the host machine's `PATH/TO/Airsim/docker/settings.json` to the docker container's `/home/airsim_user/Documents/AirSim/settings.json`.    
  * Hence, we can load any settings file by simply modifying `PATH_TO_YOUR/settings.json` by modifying the following snippets in [`run_airsim_image_source.sh`](https://github.com/Microsoft/AirSim/blob/master/docker/run_airsim_image_source.sh):
    ```
    nvidia-docker run -it \
        -v $(pwd)/settings.json:/home/airsim_user/Documents/AirSim/settings.json \
        -e SDL_VIDEODRIVER=$SDL_VIDEODRIVER_VALUE \
        -e SDL_HINT_CUDA_DEVICE='0' \
        --net=host \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        --runtime=nvidia \
        --rm \
    $DOCKER_IMAGE_NAME
    ```
