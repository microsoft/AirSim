// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from airsim_interfaces:msg/GPSYaw.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "airsim_interfaces/msg/detail/gps_yaw__rosidl_typesupport_introspection_c.h"
#include "airsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "airsim_interfaces/msg/detail/gps_yaw__functions.h"
#include "airsim_interfaces/msg/detail/gps_yaw__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void GPSYaw__rosidl_typesupport_introspection_c__GPSYaw_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  airsim_interfaces__msg__GPSYaw__init(message_memory);
}

void GPSYaw__rosidl_typesupport_introspection_c__GPSYaw_fini_function(void * message_memory)
{
  airsim_interfaces__msg__GPSYaw__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember GPSYaw__rosidl_typesupport_introspection_c__GPSYaw_message_member_array[4] = {
  {
    "latitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__GPSYaw, latitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "longitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__GPSYaw, longitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "altitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__GPSYaw, altitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__GPSYaw, yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers GPSYaw__rosidl_typesupport_introspection_c__GPSYaw_message_members = {
  "airsim_interfaces__msg",  // message namespace
  "GPSYaw",  // message name
  4,  // number of fields
  sizeof(airsim_interfaces__msg__GPSYaw),
  GPSYaw__rosidl_typesupport_introspection_c__GPSYaw_message_member_array,  // message members
  GPSYaw__rosidl_typesupport_introspection_c__GPSYaw_init_function,  // function to initialize message memory (memory has to be allocated)
  GPSYaw__rosidl_typesupport_introspection_c__GPSYaw_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t GPSYaw__rosidl_typesupport_introspection_c__GPSYaw_message_type_support_handle = {
  0,
  &GPSYaw__rosidl_typesupport_introspection_c__GPSYaw_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_airsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, msg, GPSYaw)() {
  if (!GPSYaw__rosidl_typesupport_introspection_c__GPSYaw_message_type_support_handle.typesupport_identifier) {
    GPSYaw__rosidl_typesupport_introspection_c__GPSYaw_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &GPSYaw__rosidl_typesupport_introspection_c__GPSYaw_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
