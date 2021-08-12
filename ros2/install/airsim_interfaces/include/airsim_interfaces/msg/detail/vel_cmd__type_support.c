// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from airsim_interfaces:msg/VelCmd.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "airsim_interfaces/msg/detail/vel_cmd__rosidl_typesupport_introspection_c.h"
#include "airsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "airsim_interfaces/msg/detail/vel_cmd__functions.h"
#include "airsim_interfaces/msg/detail/vel_cmd__struct.h"


// Include directives for member types
// Member `twist`
#include "geometry_msgs/msg/twist.h"
// Member `twist`
#include "geometry_msgs/msg/detail/twist__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void VelCmd__rosidl_typesupport_introspection_c__VelCmd_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  airsim_interfaces__msg__VelCmd__init(message_memory);
}

void VelCmd__rosidl_typesupport_introspection_c__VelCmd_fini_function(void * message_memory)
{
  airsim_interfaces__msg__VelCmd__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember VelCmd__rosidl_typesupport_introspection_c__VelCmd_message_member_array[1] = {
  {
    "twist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__VelCmd, twist),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers VelCmd__rosidl_typesupport_introspection_c__VelCmd_message_members = {
  "airsim_interfaces__msg",  // message namespace
  "VelCmd",  // message name
  1,  // number of fields
  sizeof(airsim_interfaces__msg__VelCmd),
  VelCmd__rosidl_typesupport_introspection_c__VelCmd_message_member_array,  // message members
  VelCmd__rosidl_typesupport_introspection_c__VelCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  VelCmd__rosidl_typesupport_introspection_c__VelCmd_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t VelCmd__rosidl_typesupport_introspection_c__VelCmd_message_type_support_handle = {
  0,
  &VelCmd__rosidl_typesupport_introspection_c__VelCmd_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_airsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, msg, VelCmd)() {
  VelCmd__rosidl_typesupport_introspection_c__VelCmd_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  if (!VelCmd__rosidl_typesupport_introspection_c__VelCmd_message_type_support_handle.typesupport_identifier) {
    VelCmd__rosidl_typesupport_introspection_c__VelCmd_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &VelCmd__rosidl_typesupport_introspection_c__VelCmd_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
