// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:msg/Altimeter.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__ALTIMETER__BUILDER_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__ALTIMETER__BUILDER_HPP_

#include "airsim_interfaces/msg/detail/altimeter__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace airsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_Altimeter_qnh
{
public:
  explicit Init_Altimeter_qnh(::airsim_interfaces::msg::Altimeter & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::msg::Altimeter qnh(::airsim_interfaces::msg::Altimeter::_qnh_type arg)
  {
    msg_.qnh = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::msg::Altimeter msg_;
};

class Init_Altimeter_pressure
{
public:
  explicit Init_Altimeter_pressure(::airsim_interfaces::msg::Altimeter & msg)
  : msg_(msg)
  {}
  Init_Altimeter_qnh pressure(::airsim_interfaces::msg::Altimeter::_pressure_type arg)
  {
    msg_.pressure = std::move(arg);
    return Init_Altimeter_qnh(msg_);
  }

private:
  ::airsim_interfaces::msg::Altimeter msg_;
};

class Init_Altimeter_altitude
{
public:
  explicit Init_Altimeter_altitude(::airsim_interfaces::msg::Altimeter & msg)
  : msg_(msg)
  {}
  Init_Altimeter_pressure altitude(::airsim_interfaces::msg::Altimeter::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_Altimeter_pressure(msg_);
  }

private:
  ::airsim_interfaces::msg::Altimeter msg_;
};

class Init_Altimeter_header
{
public:
  Init_Altimeter_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Altimeter_altitude header(::airsim_interfaces::msg::Altimeter::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Altimeter_altitude(msg_);
  }

private:
  ::airsim_interfaces::msg::Altimeter msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::msg::Altimeter>()
{
  return airsim_interfaces::msg::builder::Init_Altimeter_header();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__ALTIMETER__BUILDER_HPP_
