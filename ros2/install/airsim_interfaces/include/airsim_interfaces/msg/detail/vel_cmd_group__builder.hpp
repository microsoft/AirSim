// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:msg/VelCmdGroup.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD_GROUP__BUILDER_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD_GROUP__BUILDER_HPP_

#include "airsim_interfaces/msg/detail/vel_cmd_group__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace airsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_VelCmdGroup_vehicle_names
{
public:
  explicit Init_VelCmdGroup_vehicle_names(::airsim_interfaces::msg::VelCmdGroup & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::msg::VelCmdGroup vehicle_names(::airsim_interfaces::msg::VelCmdGroup::_vehicle_names_type arg)
  {
    msg_.vehicle_names = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::msg::VelCmdGroup msg_;
};

class Init_VelCmdGroup_twist
{
public:
  Init_VelCmdGroup_twist()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VelCmdGroup_vehicle_names twist(::airsim_interfaces::msg::VelCmdGroup::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return Init_VelCmdGroup_vehicle_names(msg_);
  }

private:
  ::airsim_interfaces::msg::VelCmdGroup msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::msg::VelCmdGroup>()
{
  return airsim_interfaces::msg::builder::Init_VelCmdGroup_twist();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD_GROUP__BUILDER_HPP_
