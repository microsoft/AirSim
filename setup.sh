#!/usr/bin/env bash
set -e
set -x

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR" >/dev/null

downloadHighPolySuv=true
DEBUG="${DEBUG:-false}"

MIN_CMAKE_VERSION=3.10.0

function version_less_than_equal_to() {
    test "$(printf '%s\n' "$@" | sort -V | head -n 1)" = "$1"
}

# -------------------------------
# Parse arguments
# -------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --debug)
            DEBUG=true
            ;;
        --no-full-poly-car)
            downloadHighPolySuv=false
            ;;
    esac
    shift
done

# -------------------------------
# Fedora dependencies
# -------------------------------
echo "Installing Fedora dependencies..."

sudo dnf update -y
sudo dnf install -y \
    git \
    cmake \
    clang llvm \
    gcc gcc-c++ \
    make \
    wget unzip rsync \
    python3 python3-pip \
    lsb-release \
    mesa-libGL-devel mesa-libEGL-devel \
    libX11-devel libXcursor-devel libXinerama-devel libXrandr-devel libXi-devel \
    vulkan-loader vulkan-validation-layers \
    mesa-dri-drivers mesa-vulkan-drivers

# -------------------------------
# Check CMake version
# -------------------------------
cmake_ver=$(cmake --version | head -n1 | awk '{print $3}')

if version_less_than_equal_to "$cmake_ver" "$MIN_CMAKE_VERSION"; then
    echo "CMake version too old: $cmake_ver"
    echo "Please install a newer cmake manually."
    exit 1
else
    echo "CMake version OK: $cmake_ver"
fi

# -------------------------------
# Dialout group (PX4 / HIL optional)
# -------------------------------
if getent group dialout >/dev/null; then
    sudo usermod -aG dialout "$USER" || true
fi

# -------------------------------
# Download rpclib
# -------------------------------
if [ ! -d "external/rpclib/rpclib-2.3.0" ]; then
    echo "Downloading rpclib..."
    rm -rf external/rpclib
    mkdir -p external/rpclib
    wget https://github.com/rpclib/rpclib/archive/v2.3.0.zip
    unzip -q v2.3.0.zip -d external/rpclib
    rm v2.3.0.zip
fi

# -------------------------------
# Download high-poly SUV assets
# -------------------------------
if $downloadHighPolySuv; then
    SUV_DIR="Unreal/Plugins/AirSim/Content/VehicleAdv/SUV/v1.2.0"
    if [ ! -d "$SUV_DIR" ]; then
        echo "Downloading high-poly SUV assets (~37MB)..."
        rm -rf suv_download_tmp
        mkdir suv_download_tmp
        cd suv_download_tmp
        wget https://github.com/Microsoft/AirSim/releases/download/v1.2.0/car_assets.zip
        unzip -q car_assets.zip -d ../Unreal/Plugins/AirSim/Content/VehicleAdv
        cd ..
        rm -rf suv_download_tmp
    fi
else
    echo "Skipping high-poly SUV assets (--no-full-poly-car)"
fi

# -------------------------------
# Install Eigen
# -------------------------------
if [ ! -d "AirLib/deps/eigen3" ]; then
    echo "Installing Eigen..."
    wget -O eigen3.zip https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip
    unzip -q eigen3.zip -d temp_eigen
    mkdir -p AirLib/deps/eigen3
    mv temp_eigen/eigen*/Eigen AirLib/deps/eigen3
    rm -rf temp_eigen eigen3.zip
else
    echo "Eigen already installed."
fi

popd >/dev/null
set +x

echo ""
echo "************************************"
echo " AirSim setup completed successfully "
echo "************************************"

