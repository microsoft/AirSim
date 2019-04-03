# get the base directory name of the unreal binary's shell script
# we'll mount this volume while running docker container
UNREAL_BINARY_PATH=$(dirname $(readlink -f $1))
UNREAL_BINARY_SHELL_ABSPATH=$(readlink -f $1)
# this block is for running X apps in docker
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# this are the first (maximum) four arguments which the user specifies:
# ex: ./run_airsim_image.sh /PATH/TO/UnrealBinary/UnrealBinary.sh -windowed -ResX=1080 -ResY=720
# we save them in a variable right now:  
UNREAL_BINARY_COMMAND="$UNREAL_BINARY_SHELL_ABSPATH $2 $3 $4"

# now let's check if user specified  an "-- headless" parameter in the end
# we'll set SDL_VIDEODRIVER_VALUE to '' if it wasn't specified, 'offscreen' if it was
SDL_VIDEODRIVER_VALUE='';
while [ -n "$1" ]; do
    case "$1" in
    --)
        shift 
        break
        ;; 
    esac
    shift
done
 
for param in $@; do
    case "$param" in
    headless) SDL_VIDEODRIVER_VALUE='offscreen' ;;
    esac
done

# now, let's mount the user directory which points to the unreal binary (UNREAL_BINARY_PATH)
# set the environment varible SDL_VIDEODRIVER to SDL_VIDEODRIVER_VALUE
# and tell the docker container to execute UNREAL_BINARY_COMMAND
nvidia-docker run -it \
    -v $(pwd)/settings.json:/home/airsim_user/Documents/AirSim/settings.json \
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
