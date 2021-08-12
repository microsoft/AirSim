// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from airsim_interfaces:msg/VelCmdGroup.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "airsim_interfaces/msg/detail/vel_cmd_group__rosidl_typesupport_introspection_c.h"
#include "airsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "airsim_interfaces/msg/detail/vel_cmd_group__functions.h"
#include "airsim_interfaces/msg/detail/vel_cmd_group__struct.h"


// Include directives for member types
// Member `twist`
#include "geometry_msgs/msg/twist.h"
// Member `twist`
#include "geometry_msgs/msg/detail/twist__rosidl_typesupport_introspection_c.h"
// Member `vehicle_names`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void VelCmdGroup__rosidl_typesupport_introspection_c__VelCmdGroup_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  airsim_interfaces__msg__VelCmdGroup__init(message_memory);
}

void VelCmdGroup__rosidl_typesupport_introspection_c__VelCmdGroup_fini_function(void * message_memory)
{
  airsim_interfaces__msg__VelCmdGroup__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember VelCmdGroup__rosidl_typesupport_introspection_c__VelCmdGroup_message_member_array[2] = {
  {
    "twist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__VelCmdGroup, twist),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vehicle_names",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__VelCmdGroup, vehicle_names),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers VelCmdGroup__rosidl_typesupport_introspection_c__VelCmdGroup_message_members = {
  "airsim_interfaces__msg",  // message namespace
  "VelCmdGroup",  // message name
  2,  // number of fields
  sizeof(airsim_interfaces__msg__VelCmdGroup),
  VelCmdGroup__rosidl_typesupport_introspection_c__VelCmdGroup_message_member_array,  // message members
  VelCmdGroup__rosidl_typesupport_introspection_c__VelCmdGroup_init_function,  // function to initialize message memory (memory has to be allocated)
  VelCmdGroup__rosidl_typesupport_introspection_c__VelCmdGroup_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t VelCmdGroup__rosidl_typesupport_introspection_c__VelCmdGroup_message_type_support_handle = {
  0,
  &VelCmdGroup__rosidl_typesupport_introspection_c__VelCmdGroup_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_airsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, msg, VelCmdGroup)() {
  VelCmdGroup__rosidl_typesupport_introspection_c__VelCmdGroup_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  if (!VelCmdGroup__rosidl_typesupport_introspection_c__VelCmdGroup_message_type_support_handle.typesupport_identifier) {
    VelCmdGroup__rosidl_typesupport_introspection_c__VelCmdGroup_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &VelCmdGroup__rosidl_typesupport_introspection_c__VelCmdGroup_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
