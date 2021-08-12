// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:srv/LandGroup.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__LAND_GROUP__BUILDER_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__LAND_GROUP__BUILDER_HPP_

#include "airsim_interfaces/srv/detail/land_group__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace airsim_interfaces
{

namespace srv
{

namespace builder
{

class Init_LandGroup_Request_wait_on_last_task
{
public:
  explicit Init_LandGroup_Request_wait_on_last_task(::airsim_interfaces::srv::LandGroup_Request & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::srv::LandGroup_Request wait_on_last_task(::airsim_interfaces::srv::LandGroup_Request::_wait_on_last_task_type arg)
  {
    msg_.wait_on_last_task = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::srv::LandGroup_Request msg_;
};

class Init_LandGroup_Request_vehicle_names
{
public:
  Init_LandGroup_Request_vehicle_names()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LandGroup_Request_wait_on_last_task vehicle_names(::airsim_interfaces::srv::LandGroup_Request::_vehicle_names_type arg)
  {
    msg_.vehicle_names = std::move(arg);
    return Init_LandGroup_Request_wait_on_last_task(msg_);
  }

private:
  ::airsim_interfaces::srv::LandGroup_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::srv::LandGroup_Request>()
{
  return airsim_interfaces::srv::builder::Init_LandGroup_Request_vehicle_names();
}

}  // namespace airsim_interfaces


namespace airsim_interfaces
{

namespace srv
{

namespace builder
{

class Init_LandGroup_Response_success
{
public:
  Init_LandGroup_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::airsim_interfaces::srv::LandGroup_Response success(::airsim_interfaces::srv::LandGroup_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::srv::LandGroup_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::srv::LandGroup_Response>()
{
  return airsim_interfaces::srv::builder::Init_LandGroup_Response_success();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__LAND_GROUP__BUILDER_HPP_
