// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from airsim_interfaces:srv/TakeoffGroup.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "airsim_interfaces/srv/detail/takeoff_group__rosidl_typesupport_introspection_c.h"
#include "airsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "airsim_interfaces/srv/detail/takeoff_group__functions.h"
#include "airsim_interfaces/srv/detail/takeoff_group__struct.h"


// Include directives for member types
// Member `vehicle_names`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void TakeoffGroup_Request__rosidl_typesupport_introspection_c__TakeoffGroup_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  airsim_interfaces__srv__TakeoffGroup_Request__init(message_memory);
}

void TakeoffGroup_Request__rosidl_typesupport_introspection_c__TakeoffGroup_Request_fini_function(void * message_memory)
{
  airsim_interfaces__srv__TakeoffGroup_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TakeoffGroup_Request__rosidl_typesupport_introspection_c__TakeoffGroup_Request_message_member_array[2] = {
  {
    "vehicle_names",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__srv__TakeoffGroup_Request, vehicle_names),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "wait_on_last_task",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__srv__TakeoffGroup_Request, wait_on_last_task),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TakeoffGroup_Request__rosidl_typesupport_introspection_c__TakeoffGroup_Request_message_members = {
  "airsim_interfaces__srv",  // message namespace
  "TakeoffGroup_Request",  // message name
  2,  // number of fields
  sizeof(airsim_interfaces__srv__TakeoffGroup_Request),
  TakeoffGroup_Request__rosidl_typesupport_introspection_c__TakeoffGroup_Request_message_member_array,  // message members
  TakeoffGroup_Request__rosidl_typesupport_introspection_c__TakeoffGroup_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  TakeoffGroup_Request__rosidl_typesupport_introspection_c__TakeoffGroup_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TakeoffGroup_Request__rosidl_typesupport_introspection_c__TakeoffGroup_Request_message_type_support_handle = {
  0,
  &TakeoffGroup_Request__rosidl_typesupport_introspection_c__TakeoffGroup_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_airsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, srv, TakeoffGroup_Request)() {
  if (!TakeoffGroup_Request__rosidl_typesupport_introspection_c__TakeoffGroup_Request_message_type_support_handle.typesupport_identifier) {
    TakeoffGroup_Request__rosidl_typesupport_introspection_c__TakeoffGroup_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TakeoffGroup_Request__rosidl_typesupport_introspection_c__TakeoffGroup_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "airsim_interfaces/srv/detail/takeoff_group__rosidl_typesupport_introspection_c.h"
// already included above
// #include "airsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "airsim_interfaces/srv/detail/takeoff_group__functions.h"
// already included above
// #include "airsim_interfaces/srv/detail/takeoff_group__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void TakeoffGroup_Response__rosidl_typesupport_introspection_c__TakeoffGroup_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  airsim_interfaces__srv__TakeoffGroup_Response__init(message_memory);
}

void TakeoffGroup_Response__rosidl_typesupport_introspection_c__TakeoffGroup_Response_fini_function(void * message_memory)
{
  airsim_interfaces__srv__TakeoffGroup_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TakeoffGroup_Response__rosidl_typesupport_introspection_c__TakeoffGroup_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__srv__TakeoffGroup_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TakeoffGroup_Response__rosidl_typesupport_introspection_c__TakeoffGroup_Response_message_members = {
  "airsim_interfaces__srv",  // message namespace
  "TakeoffGroup_Response",  // message name
  1,  // number of fields
  sizeof(airsim_interfaces__srv__TakeoffGroup_Response),
  TakeoffGroup_Response__rosidl_typesupport_introspection_c__TakeoffGroup_Response_message_member_array,  // message members
  TakeoffGroup_Response__rosidl_typesupport_introspection_c__TakeoffGroup_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  TakeoffGroup_Response__rosidl_typesupport_introspection_c__TakeoffGroup_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TakeoffGroup_Response__rosidl_typesupport_introspection_c__TakeoffGroup_Response_message_type_support_handle = {
  0,
  &TakeoffGroup_Response__rosidl_typesupport_introspection_c__TakeoffGroup_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_airsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, srv, TakeoffGroup_Response)() {
  if (!TakeoffGroup_Response__rosidl_typesupport_introspection_c__TakeoffGroup_Response_message_type_support_handle.typesupport_identifier) {
    TakeoffGroup_Response__rosidl_typesupport_introspection_c__TakeoffGroup_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TakeoffGroup_Response__rosidl_typesupport_introspection_c__TakeoffGroup_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "airsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "airsim_interfaces/srv/detail/takeoff_group__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers airsim_interfaces__srv__detail__takeoff_group__rosidl_typesupport_introspection_c__TakeoffGroup_service_members = {
  "airsim_interfaces__srv",  // service namespace
  "TakeoffGroup",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // airsim_interfaces__srv__detail__takeoff_group__rosidl_typesupport_introspection_c__TakeoffGroup_Request_message_type_support_handle,
  NULL  // response message
  // airsim_interfaces__srv__detail__takeoff_group__rosidl_typesupport_introspection_c__TakeoffGroup_Response_message_type_support_handle
};

static rosidl_service_type_support_t airsim_interfaces__srv__detail__takeoff_group__rosidl_typesupport_introspection_c__TakeoffGroup_service_type_support_handle = {
  0,
  &airsim_interfaces__srv__detail__takeoff_group__rosidl_typesupport_introspection_c__TakeoffGroup_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, srv, TakeoffGroup_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, srv, TakeoffGroup_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_airsim_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, srv, TakeoffGroup)() {
  if (!airsim_interfaces__srv__detail__takeoff_group__rosidl_typesupport_introspection_c__TakeoffGroup_service_type_support_handle.typesupport_identifier) {
    airsim_interfaces__srv__detail__takeoff_group__rosidl_typesupport_introspection_c__TakeoffGroup_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)airsim_interfaces__srv__detail__takeoff_group__rosidl_typesupport_introspection_c__TakeoffGroup_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, srv, TakeoffGroup_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, srv, TakeoffGroup_Response)()->data;
  }

  return &airsim_interfaces__srv__detail__takeoff_group__rosidl_typesupport_introspection_c__TakeoffGroup_service_type_support_handle;
}
