// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from airsim_interfaces:msg/GimbalAngleEulerCmd.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "airsim_interfaces/msg/detail/gimbal_angle_euler_cmd__rosidl_typesupport_introspection_c.h"
#include "airsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "airsim_interfaces/msg/detail/gimbal_angle_euler_cmd__functions.h"
#include "airsim_interfaces/msg/detail/gimbal_angle_euler_cmd__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `camera_name`
// Member `vehicle_name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void GimbalAngleEulerCmd__rosidl_typesupport_introspection_c__GimbalAngleEulerCmd_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  airsim_interfaces__msg__GimbalAngleEulerCmd__init(message_memory);
}

void GimbalAngleEulerCmd__rosidl_typesupport_introspection_c__GimbalAngleEulerCmd_fini_function(void * message_memory)
{
  airsim_interfaces__msg__GimbalAngleEulerCmd__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember GimbalAngleEulerCmd__rosidl_typesupport_introspection_c__GimbalAngleEulerCmd_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__GimbalAngleEulerCmd, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "camera_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__GimbalAngleEulerCmd, camera_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vehicle_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__GimbalAngleEulerCmd, vehicle_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "roll",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__GimbalAngleEulerCmd, roll),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pitch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__GimbalAngleEulerCmd, pitch),  // bytes offset in struct
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
    offsetof(airsim_interfaces__msg__GimbalAngleEulerCmd, yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers GimbalAngleEulerCmd__rosidl_typesupport_introspection_c__GimbalAngleEulerCmd_message_members = {
  "airsim_interfaces__msg",  // message namespace
  "GimbalAngleEulerCmd",  // message name
  6,  // number of fields
  sizeof(airsim_interfaces__msg__GimbalAngleEulerCmd),
  GimbalAngleEulerCmd__rosidl_typesupport_introspection_c__GimbalAngleEulerCmd_message_member_array,  // message members
  GimbalAngleEulerCmd__rosidl_typesupport_introspection_c__GimbalAngleEulerCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  GimbalAngleEulerCmd__rosidl_typesupport_introspection_c__GimbalAngleEulerCmd_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t GimbalAngleEulerCmd__rosidl_typesupport_introspection_c__GimbalAngleEulerCmd_message_type_support_handle = {
  0,
  &GimbalAngleEulerCmd__rosidl_typesupport_introspection_c__GimbalAngleEulerCmd_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_airsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, msg, GimbalAngleEulerCmd)() {
  GimbalAngleEulerCmd__rosidl_typesupport_introspection_c__GimbalAngleEulerCmd_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!GimbalAngleEulerCmd__rosidl_typesupport_introspection_c__GimbalAngleEulerCmd_message_type_support_handle.typesupport_identifier) {
    GimbalAngleEulerCmd__rosidl_typesupport_introspection_c__GimbalAngleEulerCmd_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &GimbalAngleEulerCmd__rosidl_typesupport_introspection_c__GimbalAngleEulerCmd_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
