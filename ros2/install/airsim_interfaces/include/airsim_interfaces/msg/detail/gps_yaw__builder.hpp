// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:msg/GPSYaw.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__GPS_YAW__BUILDER_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__GPS_YAW__BUILDER_HPP_

#include "airsim_interfaces/msg/detail/gps_yaw__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace airsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_GPSYaw_yaw
{
public:
  explicit Init_GPSYaw_yaw(::airsim_interfaces::msg::GPSYaw & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::msg::GPSYaw yaw(::airsim_interfaces::msg::GPSYaw::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::msg::GPSYaw msg_;
};

class Init_GPSYaw_altitude
{
public:
  explicit Init_GPSYaw_altitude(::airsim_interfaces::msg::GPSYaw & msg)
  : msg_(msg)
  {}
  Init_GPSYaw_yaw altitude(::airsim_interfaces::msg::GPSYaw::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_GPSYaw_yaw(msg_);
  }

private:
  ::airsim_interfaces::msg::GPSYaw msg_;
};

class Init_GPSYaw_longitude
{
public:
  explicit Init_GPSYaw_longitude(::airsim_interfaces::msg::GPSYaw & msg)
  : msg_(msg)
  {}
  Init_GPSYaw_altitude longitude(::airsim_interfaces::msg::GPSYaw::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_GPSYaw_altitude(msg_);
  }

private:
  ::airsim_interfaces::msg::GPSYaw msg_;
};

class Init_GPSYaw_latitude
{
public:
  Init_GPSYaw_latitude()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GPSYaw_longitude latitude(::airsim_interfaces::msg::GPSYaw::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_GPSYaw_longitude(msg_);
  }

private:
  ::airsim_interfaces::msg::GPSYaw msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::msg::GPSYaw>()
{
  return airsim_interfaces::msg::builder::Init_GPSYaw_latitude();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__GPS_YAW__BUILDER_HPP_
