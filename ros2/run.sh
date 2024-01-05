#!/usr/bin/env bash

set -euxo

SCRIPT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")

BUILD="false"
IMAGE_NAME="airsim-ros2"
CONTAINER_NAME="airsim-ros2"


function usage() {
    echo "Usage: ${0} [ -b ]" 1>&2
}

function oops() {
    usage
    exit 1
}

function parse_args() {
    local OPTIND

    source_getopts_long

    while getopts_long "b build" option "${@}"; do
        echo "option: ${option}"
        case "${option}" in
            "b" | "build")  
                BUILD="true"
                ;;
            ":")
                echo "Error: -${OPTARG} requires an argument."
                oops
                ;;
            *)
                oops
                ;;
        esac
    done
}

function ensure_build_airsim() {
    set +e
    docker image inspect "${IMAGE_NAME}" > /dev/null
    if [[ "$?" -ne 0 ]] || [[ "${BUILD}" = "true" ]]; then
        set -e
        docker pull ros:iron-ros-base
        docker build \
            --progress plain \
            -f "${SCRIPT_DIR}/Dockerfile" \
            -t "${IMAGE_NAME}" \
            "${SCRIPT_DIR}/../"
    fi
    set -e
}

function ensure_getopts_long() {
    if [ ! -d "${SCRIPT_DIR}/getopts_long" ]; then
        git clone https://github.com/UrsaDK/getopts_long.git "${SCRIPT_DIR}/getopts_long"
    fi
}

function source_getopts_long() {
    ensure_getopts_long
    source "${SCRIPT_DIR}/getopts_long/lib/getopts_long.bash"
}

function run_airsim() {
    ensure_build_airsim

    docker run --rm -it \
        --name ${CONTAINER_NAME} \
        --network host \
        "${IMAGE_NAME}"
}


parse_args "${@}"

shift $(( OPTIND - 1 ))

run_airsim
