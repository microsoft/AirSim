// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from airsim_interfaces:msg/Altimeter.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "airsim_interfaces/msg/detail/altimeter__rosidl_typesupport_introspection_c.h"
#include "airsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "airsim_interfaces/msg/detail/altimeter__functions.h"
#include "airsim_interfaces/msg/detail/altimeter__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Altimeter__rosidl_typesupport_introspection_c__Altimeter_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  airsim_interfaces__msg__Altimeter__init(message_memory);
}

void Altimeter__rosidl_typesupport_introspection_c__Altimeter_fini_function(void * message_memory)
{
  airsim_interfaces__msg__Altimeter__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Altimeter__rosidl_typesupport_introspection_c__Altimeter_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__Altimeter, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "altitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__Altimeter, altitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pressure",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__Altimeter, pressure),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "qnh",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__Altimeter, qnh),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Altimeter__rosidl_typesupport_introspection_c__Altimeter_message_members = {
  "airsim_interfaces__msg",  // message namespace
  "Altimeter",  // message name
  4,  // number of fields
  sizeof(airsim_interfaces__msg__Altimeter),
  Altimeter__rosidl_typesupport_introspection_c__Altimeter_message_member_array,  // message members
  Altimeter__rosidl_typesupport_introspection_c__Altimeter_init_function,  // function to initialize message memory (memory has to be allocated)
  Altimeter__rosidl_typesupport_introspection_c__Altimeter_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Altimeter__rosidl_typesupport_introspection_c__Altimeter_message_type_support_handle = {
  0,
  &Altimeter__rosidl_typesupport_introspection_c__Altimeter_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_airsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, msg, Altimeter)() {
  Altimeter__rosidl_typesupport_introspection_c__Altimeter_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!Altimeter__rosidl_typesupport_introspection_c__Altimeter_message_type_support_handle.typesupport_identifier) {
    Altimeter__rosidl_typesupport_introspection_c__Altimeter_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Altimeter__rosidl_typesupport_introspection_c__Altimeter_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
