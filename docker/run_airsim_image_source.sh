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
    airsim_source:4.19.2-cudagl-10.0-devel-ubuntu16.04