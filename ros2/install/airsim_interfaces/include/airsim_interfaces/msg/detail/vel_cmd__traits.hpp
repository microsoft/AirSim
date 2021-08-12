// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:msg/VelCmd.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__TRAITS_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__TRAITS_HPP_

#include "airsim_interfaces/msg/detail/vel_cmd__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<airsim_interfaces::msg::VelCmd>()
{
  return "airsim_interfaces::msg::VelCmd";
}

template<>
inline const char * name<airsim_interfaces::msg::VelCmd>()
{
  return "airsim_interfaces/msg/VelCmd";
}

template<>
struct has_fixed_size<airsim_interfaces::msg::VelCmd>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Twist>::value> {};

template<>
struct has_bounded_size<airsim_interfaces::msg::VelCmd>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Twist>::value> {};

template<>
struct is_message<airsim_interfaces::msg::VelCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__TRAITS_HPP_
