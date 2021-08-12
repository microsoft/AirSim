// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:msg/GimbalAngleEulerCmd.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_EULER_CMD__TRAITS_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_EULER_CMD__TRAITS_HPP_

#include "airsim_interfaces/msg/detail/gimbal_angle_euler_cmd__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<airsim_interfaces::msg::GimbalAngleEulerCmd>()
{
  return "airsim_interfaces::msg::GimbalAngleEulerCmd";
}

template<>
inline const char * name<airsim_interfaces::msg::GimbalAngleEulerCmd>()
{
  return "airsim_interfaces/msg/GimbalAngleEulerCmd";
}

template<>
struct has_fixed_size<airsim_interfaces::msg::GimbalAngleEulerCmd>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<airsim_interfaces::msg::GimbalAngleEulerCmd>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<airsim_interfaces::msg::GimbalAngleEulerCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_EULER_CMD__TRAITS_HPP_
