# generated from ament_package/template/package_level/local_setup.zsh.in

AMENT_SHELL=zsh

# source local_setup.sh from same directory as this file
_this_path=$(builtin cd -q "`dirname "${(%):-%N}"`" > /dev/null && pwd)
# provide AMENT_CURRENT_PREFIX to shell script
AMENT_CURRENT_PREFIX=$(builtin cd -q "`dirname "${(%):-%N}"`/../.." > /dev/null && pwd)
# store AMENT_CURRENT_PREFIX to restore it before each environment hook
_package_local_setup_AMENT_CURRENT_PREFIX=$AMENT_CURRENT_PREFIX

# function to convert array-like strings into arrays
# to wordaround SH_WORD_SPLIT not being set
ament_zsh_to_array() {
  local _listname=$1
  local _dollar="$"
  local _split="{="
  local _to_array="(\"$_dollar$_split$_listname}\")"
  eval $_listname=$_to_array
}

# trace output
if [ -n "$AMENT_TRACE_SETUP_FILES" ]; then
  echo "# . \"$_this_path/local_setup.sh\""
fi
# the package-level local_setup file unsets AMENT_CURRENT_PREFIX
. "$_this_path/local_setup.sh"
unset _this_path

# unset AMENT_ENVIRONMENT_HOOKS
# if not appending to them for return
if [ -z "$AMENT_RETURN_ENVIRONMENT_HOOKS" ]; then
  unset AMENT_ENVIRONMENT_HOOKS
fi

# restore AMENT_CURRENT_PREFIX before evaluating the environment hooks
AMENT_CURRENT_PREFIX=$_package_local_setup_AMENT_CURRENT_PREFIX
# list all environment hooks of this package

# source all shell-specific environment hooks of this package
# if not returning them
if [ -z "$AMENT_RETURN_ENVIRONMENT_HOOKS" ]; then
  _package_local_setup_IFS=$IFS
  IFS=":"
  for _hook in $AMENT_ENVIRONMENT_HOOKS; do
    # restore AMENT_CURRENT_PREFIX for each environment hook
    AMENT_CURRENT_PREFIX=$_package_local_setup_AMENT_CURRENT_PREFIX
    # restore IFS before sourcing other files
    IFS=$_package_local_setup_IFS
    . "$_hook"
  done
  unset _hook
  IFS=$_package_local_setup_IFS
  unset _package_local_setup_IFS
  unset AMENT_ENVIRONMENT_HOOKS
fi

unset _package_local_setup_AMENT_CURRENT_PREFIX
unset AMENT_CURRENT_PREFIX
