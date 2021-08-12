# generated from colcon_zsh/shell/template/prefix.zsh.em

# This script extends the environment with all packages contained in this
# prefix path.

# a zsh script is able to determine its own path if necessary
if [ -z "$COLCON_CURRENT_PREFIX" ]; then
  _colcon_prefix_zsh_COLCON_CURRENT_PREFIX="$(builtin cd -q "`dirname "${(%):-%N}"`" > /dev/null && pwd)"
else
  _colcon_prefix_zsh_COLCON_CURRENT_PREFIX="$COLCON_CURRENT_PREFIX"
fi

# function to convert array-like strings into arrays
# to workaround SH_WORD_SPLIT not being set
_colcon_prefix_zsh_convert_to_array() {
  local _listname=$1
  local _dollar="$"
  local _split="{="
  local _to_array="(\"$_dollar$_split$_listname}\")"
  eval $_listname=$_to_array
}

# function to prepend a value to a variable
# which uses colons as separators
# duplicates as well as trailing separators are avoided
# first argument: the name of the result variable
# second argument: the value to be prepended
_colcon_prefix_zsh_prepend_unique_value() {
  # arguments
  _listname="$1"
  _value="$2"

  # get values from variable
  eval _values=\"\$$_listname\"
  # backup the field separator
  _colcon_prefix_zsh_prepend_unique_value_IFS="$IFS"
  IFS=":"
  # start with the new value
  _all_values="$_value"
  # workaround SH_WORD_SPLIT not being set
  _colcon_prefix_zsh_convert_to_array _values
  # iterate over existing values in the variable
  for _item in $_values; do
    # ignore empty strings
    if [ -z "$_item" ]; then
      continue
    fi
    # ignore duplicates of _value
    if [ "$_item" = "$_value" ]; then
      continue
    fi
    # keep non-duplicate values
    _all_values="$_all_values:$_item"
  done
  unset _item
  # restore the field separator
  IFS="$_colcon_prefix_zsh_prepend_unique_value_IFS"
  unset _colcon_prefix_zsh_prepend_unique_value_IFS
  # export the updated variable
  eval export $_listname=\"$_all_values\"
  unset _all_values
  unset _values

  unset _value
  unset _listname
}

# add this prefix to the COLCON_PREFIX_PATH
_colcon_prefix_zsh_prepend_unique_value COLCON_PREFIX_PATH "$_colcon_prefix_zsh_COLCON_CURRENT_PREFIX"
unset _colcon_prefix_zsh_prepend_unique_value
unset _colcon_prefix_zsh_convert_to_array

# check environment variable for custom Python executable
if [ -n "$COLCON_PYTHON_EXECUTABLE" ]; then
  if [ ! -f "$COLCON_PYTHON_EXECUTABLE" ]; then
    echo "error: COLCON_PYTHON_EXECUTABLE '$COLCON_PYTHON_EXECUTABLE' doesn't exist"
    return 1
  fi
  _colcon_python_executable="$COLCON_PYTHON_EXECUTABLE"
else
  # try the Python executable known at configure time
  _colcon_python_executable="/usr/bin/python3"
  # if it doesn't exist try a fall back
  if [ ! -f "$_colcon_python_executable" ]; then
    if ! /usr/bin/env python3 --version > /dev/null 2> /dev/null; then
      echo "error: unable to find python3 executable"
      return 1
    fi
    _colcon_python_executable=`/usr/bin/env python3 -c "import sys; print(sys.executable)"`
  fi
fi

# function to source another script with conditional trace output
# first argument: the path of the script
_colcon_prefix_sh_source_script() {
  if [ -f "$1" ]; then
    if [ -n "$COLCON_TRACE" ]; then
      echo ". \"$1\""
    fi
    . "$1"
  else
    echo "not found: \"$1\"" 1>&2
  fi
}

# get all commands in topological order
_colcon_ordered_commands="$($_colcon_python_executable "$_colcon_prefix_zsh_COLCON_CURRENT_PREFIX/_local_setup_util_sh.py" sh zsh)"
unset _colcon_python_executable
if [ -n "$COLCON_TRACE" ]; then
  echo "Execute generated script:"
  echo "<<<"
  echo "${_colcon_ordered_commands}"
  echo ">>>"
fi
eval "${_colcon_ordered_commands}"
unset _colcon_ordered_commands

unset _colcon_prefix_sh_source_script

unset _colcon_prefix_zsh_COLCON_CURRENT_PREFIX
