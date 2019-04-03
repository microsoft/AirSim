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

nvidia-docker run -it \
    -v $(pwd)/settings.json:/home/airsim_user/Documents/AirSim/settings.json \
    -v /home/$USER:/home/$USER \
    -e SDL_VIDEODRIVER='offscreen' \
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
    /bin/bash -c "$1 $2 $3 $4"