# generated from ament_package/template/package_level/local_setup.sh.in

# since this file is sourced use either the provided AMENT_CURRENT_PREFIX
# or fall back to the destination set at configure time
: ${AMENT_CURRENT_PREFIX:="/home/alon/AirSimRos/ros/install/airsim_interfaces"}
if [ ! -d "$AMENT_CURRENT_PREFIX" ]; then
  if [ -z "$COLCON_CURRENT_PREFIX" ]; then
    echo "The compile time prefix path '$AMENT_CURRENT_PREFIX' doesn't " \
      "exist. Consider sourcing a different extension than '.sh'." 1>&2
  else
    AMENT_CURRENT_PREFIX="$COLCON_CURRENT_PREFIX"
  fi
fi

# function to append values to environment variables
# using colons as separators and avoiding leading separators
ament_append_value() {
  # arguments
  _listname="$1"
  _value="$2"
  #echo "listname $_listname"
  #eval echo "list value \$$_listname"
  #echo "value $_value"

  # avoid leading separator
  eval _values=\"\$$_listname\"
  if [ -z "$_values" ]; then
    eval export $_listname=\"$_value\"
    #eval echo "set list \$$_listname"
  else
    # field separator must not be a colon
    _ament_append_value_IFS=$IFS
    unset IFS
    eval export $_listname=\"\$$_listname:$_value\"
    #eval echo "append list \$$_listname"
    IFS=$_ament_append_value_IFS
    unset _ament_append_value_IFS
  fi
  unset _values

  unset _value
  unset _listname
}

# function to prepend non-duplicate values to environment variables
# using colons as separators and avoiding trailing separators
ament_prepend_unique_value() {
  # arguments
  _listname="$1"
  _value="$2"
  #echo "listname $_listname"
  #eval echo "list value \$$_listname"
  #echo "value $_value"

  # check if the list contains the value
  eval _values=\"\$$_listname\"
  _duplicate=
  _ament_prepend_unique_value_IFS=$IFS
  IFS=":"
  if [ "$AMENT_SHELL" = "zsh" ]; then
    ament_zsh_to_array _values
  fi
  for _item in $_values; do
    # ignore empty strings
    if [ -z "$_item" ]; then
      continue
    fi
    if [ "$_item" = "$_value" ]; then
      _duplicate=1
    fi
  done
  unset _item

  # prepend only non-duplicates
  if [ -z "$_duplicate" ]; then
    # avoid trailing separator
    if [ -z "$_values" ]; then
      eval export $_listname=\"$_value\"
      #eval echo "set list \$$_listname"
    else
      # field separator must not be a colon
      unset IFS
      eval export $_listname=\"$_value:\$$_listname\"
      #eval echo "prepend list \$$_listname"
    fi
  fi
  IFS=$_ament_prepend_unique_value_IFS
  unset _ament_prepend_unique_value_IFS
  unset _duplicate
  unset _values

  unset _value
  unset _listname
}

# unset AMENT_ENVIRONMENT_HOOKS
# if not appending to them for return
if [ -z "$AMENT_RETURN_ENVIRONMENT_HOOKS" ]; then
  unset AMENT_ENVIRONMENT_HOOKS
fi

# list all environment hooks of this package
ament_append_value AMENT_ENVIRONMENT_HOOKS "$AMENT_CURRENT_PREFIX/share/airsim_interfaces/environment/ament_prefix_path.sh"
ament_append_value AMENT_ENVIRONMENT_HOOKS "$AMENT_CURRENT_PREFIX/share/airsim_interfaces/environment/library_path.sh"
ament_append_value AMENT_ENVIRONMENT_HOOKS "$AMENT_CURRENT_PREFIX/share/airsim_interfaces/environment/path.sh"
ament_append_value AMENT_ENVIRONMENT_HOOKS "$AMENT_CURRENT_PREFIX/share/airsim_interfaces/environment/pythonpath.sh"

# source all shell-specific environment hooks of this package
# if not returning them
if [ -z "$AMENT_RETURN_ENVIRONMENT_HOOKS" ]; then
  _package_local_setup_IFS=$IFS
  IFS=":"
  if [ "$AMENT_SHELL" = "zsh" ]; then
    ament_zsh_to_array AMENT_ENVIRONMENT_HOOKS
  fi
  for _hook in $AMENT_ENVIRONMENT_HOOKS; do
    if [ -f "$_hook" ]; then
      # restore IFS before sourcing other files
      IFS=$_package_local_setup_IFS
      # trace output
      if [ -n "$AMENT_TRACE_SETUP_FILES" ]; then
        echo "# . \"$_hook\""
      fi
      . "$_hook"
    fi
  done
  unset _hook
  IFS=$_package_local_setup_IFS
  unset _package_local_setup_IFS
  unset AMENT_ENVIRONMENT_HOOKS
fi

# reset AMENT_CURRENT_PREFIX after each package
# allowing to source multiple package-level setup files
unset AMENT_CURRENT_PREFIX
