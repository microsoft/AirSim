// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from airsim_interfaces:msg/VelCmdGroup.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD_GROUP__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD_GROUP__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "airsim_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "airsim_interfaces/msg/detail/vel_cmd_group__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace airsim_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_airsim_interfaces
cdr_serialize(
  const airsim_interfaces::msg::VelCmdGroup & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_airsim_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  airsim_interfaces::msg::VelCmdGroup & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_airsim_interfaces
get_serialized_size(
  const airsim_interfaces::msg::VelCmdGroup & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_airsim_interfaces
max_serialized_size_VelCmdGroup(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace airsim_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_airsim_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, airsim_interfaces, msg, VelCmdGroup)();

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD_GROUP__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
