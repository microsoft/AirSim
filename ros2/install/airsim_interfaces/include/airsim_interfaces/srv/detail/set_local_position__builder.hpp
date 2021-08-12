// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:srv/SetLocalPosition.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__SET_LOCAL_POSITION__BUILDER_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__SET_LOCAL_POSITION__BUILDER_HPP_

#include "airsim_interfaces/srv/detail/set_local_position__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace airsim_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetLocalPosition_Request_vehicle_name
{
public:
  explicit Init_SetLocalPosition_Request_vehicle_name(::airsim_interfaces::srv::SetLocalPosition_Request & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::srv::SetLocalPosition_Request vehicle_name(::airsim_interfaces::srv::SetLocalPosition_Request::_vehicle_name_type arg)
  {
    msg_.vehicle_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::srv::SetLocalPosition_Request msg_;
};

class Init_SetLocalPosition_Request_yaw
{
public:
  explicit Init_SetLocalPosition_Request_yaw(::airsim_interfaces::srv::SetLocalPosition_Request & msg)
  : msg_(msg)
  {}
  Init_SetLocalPosition_Request_vehicle_name yaw(::airsim_interfaces::srv::SetLocalPosition_Request::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_SetLocalPosition_Request_vehicle_name(msg_);
  }

private:
  ::airsim_interfaces::srv::SetLocalPosition_Request msg_;
};

class Init_SetLocalPosition_Request_z
{
public:
  explicit Init_SetLocalPosition_Request_z(::airsim_interfaces::srv::SetLocalPosition_Request & msg)
  : msg_(msg)
  {}
  Init_SetLocalPosition_Request_yaw z(::airsim_interfaces::srv::SetLocalPosition_Request::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_SetLocalPosition_Request_yaw(msg_);
  }

private:
  ::airsim_interfaces::srv::SetLocalPosition_Request msg_;
};

class Init_SetLocalPosition_Request_y
{
public:
  explicit Init_SetLocalPosition_Request_y(::airsim_interfaces::srv::SetLocalPosition_Request & msg)
  : msg_(msg)
  {}
  Init_SetLocalPosition_Request_z y(::airsim_interfaces::srv::SetLocalPosition_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_SetLocalPosition_Request_z(msg_);
  }

private:
  ::airsim_interfaces::srv::SetLocalPosition_Request msg_;
};

class Init_SetLocalPosition_Request_x
{
public:
  Init_SetLocalPosition_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetLocalPosition_Request_y x(::airsim_interfaces::srv::SetLocalPosition_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_SetLocalPosition_Request_y(msg_);
  }

private:
  ::airsim_interfaces::srv::SetLocalPosition_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::srv::SetLocalPosition_Request>()
{
  return airsim_interfaces::srv::builder::Init_SetLocalPosition_Request_x();
}

}  // namespace airsim_interfaces


namespace airsim_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetLocalPosition_Response_message
{
public:
  explicit Init_SetLocalPosition_Response_message(::airsim_interfaces::srv::SetLocalPosition_Response & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::srv::SetLocalPosition_Response message(::airsim_interfaces::srv::SetLocalPosition_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::srv::SetLocalPosition_Response msg_;
};

class Init_SetLocalPosition_Response_success
{
public:
  Init_SetLocalPosition_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetLocalPosition_Response_message success(::airsim_interfaces::srv::SetLocalPosition_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetLocalPosition_Response_message(msg_);
  }

private:
  ::airsim_interfaces::srv::SetLocalPosition_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::srv::SetLocalPosition_Response>()
{
  return airsim_interfaces::srv::builder::Init_SetLocalPosition_Response_success();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__SET_LOCAL_POSITION__BUILDER_HPP_
