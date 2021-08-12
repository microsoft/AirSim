// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from airsim_interfaces:srv/Land.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__LAND__TRAITS_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__LAND__TRAITS_HPP_

#include "airsim_interfaces/srv/detail/land__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<airsim_interfaces::srv::Land_Request>()
{
  return "airsim_interfaces::srv::Land_Request";
}

template<>
inline const char * name<airsim_interfaces::srv::Land_Request>()
{
  return "airsim_interfaces/srv/Land_Request";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::Land_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<airsim_interfaces::srv::Land_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<airsim_interfaces::srv::Land_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<airsim_interfaces::srv::Land_Response>()
{
  return "airsim_interfaces::srv::Land_Response";
}

template<>
inline const char * name<airsim_interfaces::srv::Land_Response>()
{
  return "airsim_interfaces/srv/Land_Response";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::Land_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<airsim_interfaces::srv::Land_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<airsim_interfaces::srv::Land_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<airsim_interfaces::srv::Land>()
{
  return "airsim_interfaces::srv::Land";
}

template<>
inline const char * name<airsim_interfaces::srv::Land>()
{
  return "airsim_interfaces/srv/Land";
}

template<>
struct has_fixed_size<airsim_interfaces::srv::Land>
  : std::integral_constant<
    bool,
    has_fixed_size<airsim_interfaces::srv::Land_Request>::value &&
    has_fixed_size<airsim_interfaces::srv::Land_Response>::value
  >
{
};

template<>
struct has_bounded_size<airsim_interfaces::srv::Land>
  : std::integral_constant<
    bool,
    has_bounded_size<airsim_interfaces::srv::Land_Request>::value &&
    has_bounded_size<airsim_interfaces::srv::Land_Response>::value
  >
{
};

template<>
struct is_service<airsim_interfaces::srv::Land>
  : std::true_type
{
};

template<>
struct is_service_request<airsim_interfaces::srv::Land_Request>
  : std::true_type
{
};

template<>
struct is_service_response<airsim_interfaces::srv::Land_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__LAND__TRAITS_HPP_
