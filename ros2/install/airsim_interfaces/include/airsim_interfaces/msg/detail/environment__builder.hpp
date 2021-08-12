// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:msg/Environment.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__ENVIRONMENT__BUILDER_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__ENVIRONMENT__BUILDER_HPP_

#include "airsim_interfaces/msg/detail/environment__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace airsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_Environment_air_density
{
public:
  explicit Init_Environment_air_density(::airsim_interfaces::msg::Environment & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::msg::Environment air_density(::airsim_interfaces::msg::Environment::_air_density_type arg)
  {
    msg_.air_density = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::msg::Environment msg_;
};

class Init_Environment_temperature
{
public:
  explicit Init_Environment_temperature(::airsim_interfaces::msg::Environment & msg)
  : msg_(msg)
  {}
  Init_Environment_air_density temperature(::airsim_interfaces::msg::Environment::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_Environment_air_density(msg_);
  }

private:
  ::airsim_interfaces::msg::Environment msg_;
};

class Init_Environment_air_pressure
{
public:
  explicit Init_Environment_air_pressure(::airsim_interfaces::msg::Environment & msg)
  : msg_(msg)
  {}
  Init_Environment_temperature air_pressure(::airsim_interfaces::msg::Environment::_air_pressure_type arg)
  {
    msg_.air_pressure = std::move(arg);
    return Init_Environment_temperature(msg_);
  }

private:
  ::airsim_interfaces::msg::Environment msg_;
};

class Init_Environment_gravity
{
public:
  explicit Init_Environment_gravity(::airsim_interfaces::msg::Environment & msg)
  : msg_(msg)
  {}
  Init_Environment_air_pressure gravity(::airsim_interfaces::msg::Environment::_gravity_type arg)
  {
    msg_.gravity = std::move(arg);
    return Init_Environment_air_pressure(msg_);
  }

private:
  ::airsim_interfaces::msg::Environment msg_;
};

class Init_Environment_geo_point
{
public:
  explicit Init_Environment_geo_point(::airsim_interfaces::msg::Environment & msg)
  : msg_(msg)
  {}
  Init_Environment_gravity geo_point(::airsim_interfaces::msg::Environment::_geo_point_type arg)
  {
    msg_.geo_point = std::move(arg);
    return Init_Environment_gravity(msg_);
  }

private:
  ::airsim_interfaces::msg::Environment msg_;
};

class Init_Environment_position
{
public:
  explicit Init_Environment_position(::airsim_interfaces::msg::Environment & msg)
  : msg_(msg)
  {}
  Init_Environment_geo_point position(::airsim_interfaces::msg::Environment::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Environment_geo_point(msg_);
  }

private:
  ::airsim_interfaces::msg::Environment msg_;
};

class Init_Environment_header
{
public:
  Init_Environment_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Environment_position header(::airsim_interfaces::msg::Environment::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Environment_position(msg_);
  }

private:
  ::airsim_interfaces::msg::Environment msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::msg::Environment>()
{
  return airsim_interfaces::msg::builder::Init_Environment_header();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__ENVIRONMENT__BUILDER_HPP_
