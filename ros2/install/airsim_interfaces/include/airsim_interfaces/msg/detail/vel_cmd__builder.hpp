// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:msg/VelCmd.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__BUILDER_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__BUILDER_HPP_

#include "airsim_interfaces/msg/detail/vel_cmd__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace airsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_VelCmd_twist
{
public:
  Init_VelCmd_twist()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::airsim_interfaces::msg::VelCmd twist(::airsim_interfaces::msg::VelCmd::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::msg::VelCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::msg::VelCmd>()
{
  return airsim_interfaces::msg::builder::Init_VelCmd_twist();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__BUILDER_HPP_
