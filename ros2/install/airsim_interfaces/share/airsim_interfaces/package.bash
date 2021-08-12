# generated from colcon_bash/shell/template/package.bash.em

# This script extends the environment for this package.

# a bash script is able to determine its own path if necessary
if [ -z "$COLCON_CURRENT_PREFIX" ]; then
  # the prefix is two levels up from the package specific share directory
  _colcon_package_bash_COLCON_CURRENT_PREFIX="$(builtin cd "`dirname "${BASH_SOURCE[0]}"`/../.." > /dev/null && pwd)"
else
  _colcon_package_bash_COLCON_CURRENT_PREFIX="$COLCON_CURRENT_PREFIX"
fi

# function to source another script with conditional trace output
# first argument: the path of the script
# additional arguments: arguments to the script
_colcon_package_bash_source_script() {
  if [ -f "$1" ]; then
    if [ -n "$COLCON_TRACE" ]; then
      echo ". \"$1\""
    fi
    . "$@"
  else
    echo "not found: \"$1\"" 1>&2
  fi
}

# source sh script of this package
_colcon_package_bash_source_script "$_colcon_package_bash_COLCON_CURRENT_PREFIX/share/airsim_interfaces/package.sh"

# setting COLCON_CURRENT_PREFIX avoids determining the prefix in the sourced scripts
COLCON_CURRENT_PREFIX="$_colcon_package_bash_COLCON_CURRENT_PREFIX"

# source bash hooks
_colcon_package_bash_source_script "$COLCON_CURRENT_PREFIX/share/airsim_interfaces/local_setup.bash"

unset COLCON_CURRENT_PREFIX

unset _colcon_package_bash_source_script
unset _colcon_package_bash_COLCON_CURRENT_PREFIX
