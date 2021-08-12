# copied from ament_cmake_core/cmake/environment_hooks/environment/path.sh

if [ -d "$AMENT_CURRENT_PREFIX/bin" ]; then
  ament_prepend_unique_value PATH "$AMENT_CURRENT_PREFIX/bin"
fi
