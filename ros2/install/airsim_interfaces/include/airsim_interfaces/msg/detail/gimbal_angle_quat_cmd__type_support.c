// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from airsim_interfaces:msg/GimbalAngleQuatCmd.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "airsim_interfaces/msg/detail/gimbal_angle_quat_cmd__rosidl_typesupport_introspection_c.h"
#include "airsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "airsim_interfaces/msg/detail/gimbal_angle_quat_cmd__functions.h"
#include "airsim_interfaces/msg/detail/gimbal_angle_quat_cmd__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `camera_name`
// Member `vehicle_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `orientation`
#include "geometry_msgs/msg/quaternion.h"
// Member `orientation`
#include "geometry_msgs/msg/detail/quaternion__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void GimbalAngleQuatCmd__rosidl_typesupport_introspection_c__GimbalAngleQuatCmd_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  airsim_interfaces__msg__GimbalAngleQuatCmd__init(message_memory);
}

void GimbalAngleQuatCmd__rosidl_typesupport_introspection_c__GimbalAngleQuatCmd_fini_function(void * message_memory)
{
  airsim_interfaces__msg__GimbalAngleQuatCmd__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember GimbalAngleQuatCmd__rosidl_typesupport_introspection_c__GimbalAngleQuatCmd_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__GimbalAngleQuatCmd, header),  // bytes offset in struct
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
    offsetof(airsim_interfaces__msg__GimbalAngleQuatCmd, camera_name),  // bytes offset in struct
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
    offsetof(airsim_interfaces__msg__GimbalAngleQuatCmd, vehicle_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "orientation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__GimbalAngleQuatCmd, orientation),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers GimbalAngleQuatCmd__rosidl_typesupport_introspection_c__GimbalAngleQuatCmd_message_members = {
  "airsim_interfaces__msg",  // message namespace
  "GimbalAngleQuatCmd",  // message name
  4,  // number of fields
  sizeof(airsim_interfaces__msg__GimbalAngleQuatCmd),
  GimbalAngleQuatCmd__rosidl_typesupport_introspection_c__GimbalAngleQuatCmd_message_member_array,  // message members
  GimbalAngleQuatCmd__rosidl_typesupport_introspection_c__GimbalAngleQuatCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  GimbalAngleQuatCmd__rosidl_typesupport_introspection_c__GimbalAngleQuatCmd_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t GimbalAngleQuatCmd__rosidl_typesupport_introspection_c__GimbalAngleQuatCmd_message_type_support_handle = {
  0,
  &GimbalAngleQuatCmd__rosidl_typesupport_introspection_c__GimbalAngleQuatCmd_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_airsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, msg, GimbalAngleQuatCmd)() {
  GimbalAngleQuatCmd__rosidl_typesupport_introspection_c__GimbalAngleQuatCmd_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  GimbalAngleQuatCmd__rosidl_typesupport_introspection_c__GimbalAngleQuatCmd_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Quaternion)();
  if (!GimbalAngleQuatCmd__rosidl_typesupport_introspection_c__GimbalAngleQuatCmd_message_type_support_handle.typesupport_identifier) {
    GimbalAngleQuatCmd__rosidl_typesupport_introspection_c__GimbalAngleQuatCmd_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &GimbalAngleQuatCmd__rosidl_typesupport_introspection_c__GimbalAngleQuatCmd_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
