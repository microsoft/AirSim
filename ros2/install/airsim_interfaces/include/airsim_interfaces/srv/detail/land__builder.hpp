// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:srv/Land.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__LAND__BUILDER_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__LAND__BUILDER_HPP_

#include "airsim_interfaces/srv/detail/land__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace airsim_interfaces
{

namespace srv
{

namespace builder
{

class Init_Land_Request_wait_on_last_task
{
public:
  Init_Land_Request_wait_on_last_task()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::airsim_interfaces::srv::Land_Request wait_on_last_task(::airsim_interfaces::srv::Land_Request::_wait_on_last_task_type arg)
  {
    msg_.wait_on_last_task = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::srv::Land_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::srv::Land_Request>()
{
  return airsim_interfaces::srv::builder::Init_Land_Request_wait_on_last_task();
}

}  // namespace airsim_interfaces


namespace airsim_interfaces
{

namespace srv
{

namespace builder
{

class Init_Land_Response_success
{
public:
  Init_Land_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::airsim_interfaces::srv::Land_Response success(::airsim_interfaces::srv::Land_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::srv::Land_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::srv::Land_Response>()
{
  return airsim_interfaces::srv::builder::Init_Land_Response_success();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__LAND__BUILDER_HPP_
