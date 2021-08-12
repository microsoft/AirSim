// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:msg/VelCmdGroup.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD_GROUP__TRAITS_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD_GROUP__TRAITS_HPP_

#include "airsim_interfaces/msg/detail/vel_cmd_group__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<airsim_interfaces::msg::VelCmdGroup>()
{
  return "airsim_interfaces::msg::VelCmdGroup";
}

template<>
inline const char * name<airsim_interfaces::msg::VelCmdGroup>()
{
  return "airsim_interfaces/msg/VelCmdGroup";
}

template<>
struct has_fixed_size<airsim_interfaces::msg::VelCmdGroup>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<airsim_interfaces::msg::VelCmdGroup>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<airsim_interfaces::msg::VelCmdGroup>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD_GROUP__TRAITS_HPP_
