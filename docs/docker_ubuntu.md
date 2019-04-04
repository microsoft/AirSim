# AirSim on Docker in Linux

## Binaries
#### Requirements:
- Install [nvidia-docker2](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-2.0))

#### Build the docker image
```
$ cd Airsim/docker;
$ ./build_airsim_image.sh
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

- Run the Blocks binary inside a docker container 
    ```
    $ cd Airsim/docker;
    $ ./run_airsim_image.sh Blocks/Blocks.sh -windowed
    ```
    * Specifying Resolution   :
        ```
        $ ./run_airsim_image.sh Blocks/Blocks.sh -windowed -ResX=1080
        ## OR:
        $ ./run_airsim_image.sh Blocks/Blocks.sh -windowed -ResX=1080 -ResY=720
        ```

    * Running in Headless mode:    
        Suffix `-- headless` at the end:
        ```
        $ ./run_airsim_image.sh Blocks/Blocks.sh -- headless
        ```
- Specifying a `settings.json`:   
    Look inside [`run_airsim_image.sh`](https://github.com/Microsoft/AirSim/blob/master/docker/run_airsim_image.sh).    
    We're mapping the host machine's `PATH/TO/Airsim/docker/settings.json` to the docker containers `/home/airsim_user/Documents/AirSim/settings.json`.    
    Hence, we can load any settings file by simply modifying `PATH_TO_YOUR/settings.json` by modifying the following snippet in the end of [`run_airsim_image.sh`](https://github.com/Microsoft/AirSim/blob/master/docker/run_airsim_image.sh):

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
        airsim:cudagl-10.0-devel-ubuntu16.04 \
        /bin/bash -c "$UNREAL_BINARY_COMMAND"
    ```
 
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
   `./build_airsim_image_source.sh`

#### Running AirSim container
* `./run_airsim_image_source.sh`
* Start unreal engine:   
   `/home/ue4/UnrealEngine/Engine/Binaries/Linux/UE4Editor`

#### Packaging Unreal Environments
* Let's take the Blocks environment as an example.    
    In the following script, specify the full path to your unreal uproject file by `project` and the directory where you want the binaries to be placed by `archivedirectory` 
    ```
    $ /home/ue4/UnrealEngine/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun -platform=Linux -clientconfig=Shipping -serverconfig=Shipping -noP4 -cook -allmaps -build -stage -prereqs -pak -archive \
    -archivedirectory=/home/ue4/Binaries/Blocks/ \
    -project=/home/ue4/AirSim/Unreal/Environments/Blocks/Blocks.uproject
    ```

    This would create a Blocks binary in `/home/ue4/Binaries/Blocks/`.   
    You can test it by running `/home/ue4/Binaries/Blocks/LinuxNoEditor/Blocks.sh -windowed`   
TODO : add changes in build.cs and target.cs
