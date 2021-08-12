// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:srv/SetGPSPosition.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__SET_GPS_POSITION__BUILDER_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__SET_GPS_POSITION__BUILDER_HPP_

#include "airsim_interfaces/srv/detail/set_gps_position__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace airsim_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetGPSPosition_Request_vehicle_name
{
public:
  explicit Init_SetGPSPosition_Request_vehicle_name(::airsim_interfaces::srv::SetGPSPosition_Request & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::srv::SetGPSPosition_Request vehicle_name(::airsim_interfaces::srv::SetGPSPosition_Request::_vehicle_name_type arg)
  {
    msg_.vehicle_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::srv::SetGPSPosition_Request msg_;
};

class Init_SetGPSPosition_Request_yaw
{
public:
  explicit Init_SetGPSPosition_Request_yaw(::airsim_interfaces::srv::SetGPSPosition_Request & msg)
  : msg_(msg)
  {}
  Init_SetGPSPosition_Request_vehicle_name yaw(::airsim_interfaces::srv::SetGPSPosition_Request::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_SetGPSPosition_Request_vehicle_name(msg_);
  }

private:
  ::airsim_interfaces::srv::SetGPSPosition_Request msg_;
};

class Init_SetGPSPosition_Request_altitude
{
public:
  explicit Init_SetGPSPosition_Request_altitude(::airsim_interfaces::srv::SetGPSPosition_Request & msg)
  : msg_(msg)
  {}
  Init_SetGPSPosition_Request_yaw altitude(::airsim_interfaces::srv::SetGPSPosition_Request::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_SetGPSPosition_Request_yaw(msg_);
  }

private:
  ::airsim_interfaces::srv::SetGPSPosition_Request msg_;
};

class Init_SetGPSPosition_Request_longitude
{
public:
  explicit Init_SetGPSPosition_Request_longitude(::airsim_interfaces::srv::SetGPSPosition_Request & msg)
  : msg_(msg)
  {}
  Init_SetGPSPosition_Request_altitude longitude(::airsim_interfaces::srv::SetGPSPosition_Request::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_SetGPSPosition_Request_altitude(msg_);
  }

private:
  ::airsim_interfaces::srv::SetGPSPosition_Request msg_;
};

class Init_SetGPSPosition_Request_latitude
{
public:
  Init_SetGPSPosition_Request_latitude()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetGPSPosition_Request_longitude latitude(::airsim_interfaces::srv::SetGPSPosition_Request::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_SetGPSPosition_Request_longitude(msg_);
  }

private:
  ::airsim_interfaces::srv::SetGPSPosition_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::srv::SetGPSPosition_Request>()
{
  return airsim_interfaces::srv::builder::Init_SetGPSPosition_Request_latitude();
}

}  // namespace airsim_interfaces


namespace airsim_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetGPSPosition_Response_success
{
public:
  Init_SetGPSPosition_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::airsim_interfaces::srv::SetGPSPosition_Response success(::airsim_interfaces::srv::SetGPSPosition_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::srv::SetGPSPosition_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::srv::SetGPSPosition_Response>()
{
  return airsim_interfaces::srv::builder::Init_SetGPSPosition_Response_success();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__SET_GPS_POSITION__BUILDER_HPP_
