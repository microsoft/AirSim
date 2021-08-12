// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:msg/GPSYaw.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__GPS_YAW__TRAITS_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__GPS_YAW__TRAITS_HPP_

#include "airsim_interfaces/msg/detail/gps_yaw__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<airsim_interfaces::msg::GPSYaw>()
{
  return "airsim_interfaces::msg::GPSYaw";
}

template<>
inline const char * name<airsim_interfaces::msg::GPSYaw>()
{
  return "airsim_interfaces/msg/GPSYaw";
}

template<>
struct has_fixed_size<airsim_interfaces::msg::GPSYaw>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<airsim_interfaces::msg::GPSYaw>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<airsim_interfaces::msg::GPSYaw>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__GPS_YAW__TRAITS_HPP_
