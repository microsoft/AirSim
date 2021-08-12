// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from airsim_interfaces:srv/Land.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "airsim_interfaces/srv/detail/land__rosidl_typesupport_introspection_c.h"
#include "airsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "airsim_interfaces/srv/detail/land__functions.h"
#include "airsim_interfaces/srv/detail/land__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void Land_Request__rosidl_typesupport_introspection_c__Land_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  airsim_interfaces__srv__Land_Request__init(message_memory);
}

void Land_Request__rosidl_typesupport_introspection_c__Land_Request_fini_function(void * message_memory)
{
  airsim_interfaces__srv__Land_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Land_Request__rosidl_typesupport_introspection_c__Land_Request_message_member_array[1] = {
  {
    "wait_on_last_task",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__srv__Land_Request, wait_on_last_task),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Land_Request__rosidl_typesupport_introspection_c__Land_Request_message_members = {
  "airsim_interfaces__srv",  // message namespace
  "Land_Request",  // message name
  1,  // number of fields
  sizeof(airsim_interfaces__srv__Land_Request),
  Land_Request__rosidl_typesupport_introspection_c__Land_Request_message_member_array,  // message members
  Land_Request__rosidl_typesupport_introspection_c__Land_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  Land_Request__rosidl_typesupport_introspection_c__Land_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Land_Request__rosidl_typesupport_introspection_c__Land_Request_message_type_support_handle = {
  0,
  &Land_Request__rosidl_typesupport_introspection_c__Land_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_airsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, srv, Land_Request)() {
  if (!Land_Request__rosidl_typesupport_introspection_c__Land_Request_message_type_support_handle.typesupport_identifier) {
    Land_Request__rosidl_typesupport_introspection_c__Land_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Land_Request__rosidl_typesupport_introspection_c__Land_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "airsim_interfaces/srv/detail/land__rosidl_typesupport_introspection_c.h"
// already included above
// #include "airsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "airsim_interfaces/srv/detail/land__functions.h"
// already included above
// #include "airsim_interfaces/srv/detail/land__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void Land_Response__rosidl_typesupport_introspection_c__Land_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  airsim_interfaces__srv__Land_Response__init(message_memory);
}

void Land_Response__rosidl_typesupport_introspection_c__Land_Response_fini_function(void * message_memory)
{
  airsim_interfaces__srv__Land_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Land_Response__rosidl_typesupport_introspection_c__Land_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__srv__Land_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Land_Response__rosidl_typesupport_introspection_c__Land_Response_message_members = {
  "airsim_interfaces__srv",  // message namespace
  "Land_Response",  // message name
  1,  // number of fields
  sizeof(airsim_interfaces__srv__Land_Response),
  Land_Response__rosidl_typesupport_introspection_c__Land_Response_message_member_array,  // message members
  Land_Response__rosidl_typesupport_introspection_c__Land_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  Land_Response__rosidl_typesupport_introspection_c__Land_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Land_Response__rosidl_typesupport_introspection_c__Land_Response_message_type_support_handle = {
  0,
  &Land_Response__rosidl_typesupport_introspection_c__Land_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_airsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, srv, Land_Response)() {
  if (!Land_Response__rosidl_typesupport_introspection_c__Land_Response_message_type_support_handle.typesupport_identifier) {
    Land_Response__rosidl_typesupport_introspection_c__Land_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Land_Response__rosidl_typesupport_introspection_c__Land_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "airsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "airsim_interfaces/srv/detail/land__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers airsim_interfaces__srv__detail__land__rosidl_typesupport_introspection_c__Land_service_members = {
  "airsim_interfaces__srv",  // service namespace
  "Land",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // airsim_interfaces__srv__detail__land__rosidl_typesupport_introspection_c__Land_Request_message_type_support_handle,
  NULL  // response message
  // airsim_interfaces__srv__detail__land__rosidl_typesupport_introspection_c__Land_Response_message_type_support_handle
};

static rosidl_service_type_support_t airsim_interfaces__srv__detail__land__rosidl_typesupport_introspection_c__Land_service_type_support_handle = {
  0,
  &airsim_interfaces__srv__detail__land__rosidl_typesupport_introspection_c__Land_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, srv, Land_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, srv, Land_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_airsim_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, srv, Land)() {
  if (!airsim_interfaces__srv__detail__land__rosidl_typesupport_introspection_c__Land_service_type_support_handle.typesupport_identifier) {
    airsim_interfaces__srv__detail__land__rosidl_typesupport_introspection_c__Land_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)airsim_interfaces__srv__detail__land__rosidl_typesupport_introspection_c__Land_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, srv, Land_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, srv, Land_Response)()->data;
  }

  return &airsim_interfaces__srv__detail__land__rosidl_typesupport_introspection_c__Land_service_type_support_handle;
}
