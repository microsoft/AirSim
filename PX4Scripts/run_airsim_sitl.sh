instance_num=0
[ -n "$1" ] && instance_num="$1"
export PX4_SIM_MODEL=iris
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR=$PARENT_DIR/Firmware/ROMFS/px4fmu_common
instance_path=$PARENT_DIR/Firmware/build/px4_sitl_default
BIN_DIR=$PARENT_DIR/Firmware/build/px4_sitl_default/bin/px4
TEST_DATA=$PARENT_DIR/Firmware/test_data
working_dir="$instance_path/instance_$instance_num"
[ ! -d "$working_dir" ] && mkdir -p "$working_dir"
pushd "$working_dir" &>/dev/null
$BIN_DIR -i $instance_num $BUILD_DIR -s "etc/init.d-posix/rcS" -t $TEST_DATA
