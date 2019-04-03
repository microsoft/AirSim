# AirSim on Docker in Linux

## Binaries

### Requirements:
- Install [nvidia-docker2](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-2.0))

### Setup
- Build the docker image
```
$ cd Airsim/docker;
$ ./build_airsim_image.sh
```
Verify you have an image by:
```
$ docker images | grep airsim
```

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
You can also add optional resolution parametere:
```
$ ./run_airsim_image.sh Blocks/Blocks.sh -windowed -ResX=1080
## OR:
$ ./run_airsim_image.sh Blocks/Blocks.sh -windowed -ResX=1080 -ResY=720
```

You can also run the binary in headless mode by suffixing `-- headless` at the end:
```
$ ./run_airsim_image.sh Blocks/Blocks.sh -- headless
```

# Specifying a `settings.json`:
Look inside `run_airsim_image.sh`.    
We're mapping the host machine's `Airsim/docker/settings.json` to the docker containers `/home/airsim_user/Documents/AirSim/settings.json`.    
Hence, we can load any settings file by simply modifying `PATH_TO_YOUR/settings.json`:

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
 
