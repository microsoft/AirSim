macro(BuildAirLib)
include_directories(
  ${AIRSIM_ROOT}/AirLib/
  ${AIRSIM_ROOT}/AirLib/include
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${RPC_LIB_INCLUDES}
)

file(GLOB_RECURSE AIRLIB_LIBRARY_SOURCE_FILES
  ${AIRSIM_ROOT}/AirLib/src/api/*.cpp
  ${AIRSIM_ROOT}/AirLib/src/common/common_utils/*.cpp
  ${AIRSIM_ROOT}/AirLib/src/safety/*.cpp
  ${AIRSIM_ROOT}/AirLib/src/vehicles/car/*.cpp
  ${AIRSIM_ROOT}/AirLib/src/vehicles/multirotor/*.cpp
)

endmacro(BuildAirLib)