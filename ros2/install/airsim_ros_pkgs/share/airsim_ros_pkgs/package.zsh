# generated from colcon_zsh/shell/template/package.zsh.em

# This script extends the environment for this package.

# a zsh script is able to determine its own path if necessary
if [ -z "$COLCON_CURRENT_PREFIX" ]; then
  # the prefix is two levels up from the package specific share directory
  _colcon_package_zsh_COLCON_CURRENT_PREFIX="$(builtin cd -q "`dirname "${(%):-%N}"`/../.." > /dev/null && pwd)"
else
  _colcon_package_zsh_COLCON_CURRENT_PREFIX="$COLCON_CURRENT_PREFIX"
fi

# function to source another script with conditional trace output
# first argument: the path of the script
# additional arguments: arguments to the script
_colcon_package_zsh_source_script() {
  if [ -f "$1" ]; then
    if [ -n "$COLCON_TRACE" ]; then
      echo ". \"$1\""
    fi
    . "$@"
  else
    echo "not found: \"$1\"" 1>&2
  fi
}

# function to convert array-like strings into arrays
# to workaround SH_WORD_SPLIT not being set
colcon_zsh_convert_to_array() {
  local _listname=$1
  local _dollar="$"
  local _split="{="
  local _to_array="(\"$_dollar$_split$_listname}\")"
  eval $_listname=$_to_array
}

# source sh script of this package
_colcon_package_zsh_source_script "$_colcon_package_zsh_COLCON_CURRENT_PREFIX/share/airsim_ros_pkgs/package.sh"
unset convert_zsh_to_array

# setting COLCON_CURRENT_PREFIX avoids determining the prefix in the sourced scripts
COLCON_CURRENT_PREFIX="$_colcon_package_zsh_COLCON_CURRENT_PREFIX"

# source zsh hooks
_colcon_package_zsh_source_script "$COLCON_CURRENT_PREFIX/share/airsim_ros_pkgs/local_setup.zsh"

unset COLCON_CURRENT_PREFIX

unset _colcon_package_zsh_source_script
unset _colcon_package_zsh_COLCON_CURRENT_PREFIX
