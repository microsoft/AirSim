// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:msg/GimbalAngleQuatCmd.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_QUAT_CMD__TRAITS_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_QUAT_CMD__TRAITS_HPP_

#include "airsim_interfaces/msg/detail/gimbal_angle_quat_cmd__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<airsim_interfaces::msg::GimbalAngleQuatCmd>()
{
  return "airsim_interfaces::msg::GimbalAngleQuatCmd";
}

template<>
inline const char * name<airsim_interfaces::msg::GimbalAngleQuatCmd>()
{
  return "airsim_interfaces/msg/GimbalAngleQuatCmd";
}

template<>
struct has_fixed_size<airsim_interfaces::msg::GimbalAngleQuatCmd>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<airsim_interfaces::msg::GimbalAngleQuatCmd>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<airsim_interfaces::msg::GimbalAngleQuatCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_QUAT_CMD__TRAITS_HPP_
