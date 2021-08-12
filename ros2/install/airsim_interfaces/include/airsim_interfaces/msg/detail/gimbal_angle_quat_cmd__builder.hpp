// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:msg/GimbalAngleQuatCmd.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_QUAT_CMD__BUILDER_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_QUAT_CMD__BUILDER_HPP_

#include "airsim_interfaces/msg/detail/gimbal_angle_quat_cmd__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace airsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_GimbalAngleQuatCmd_orientation
{
public:
  explicit Init_GimbalAngleQuatCmd_orientation(::airsim_interfaces::msg::GimbalAngleQuatCmd & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::msg::GimbalAngleQuatCmd orientation(::airsim_interfaces::msg::GimbalAngleQuatCmd::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::msg::GimbalAngleQuatCmd msg_;
};

class Init_GimbalAngleQuatCmd_vehicle_name
{
public:
  explicit Init_GimbalAngleQuatCmd_vehicle_name(::airsim_interfaces::msg::GimbalAngleQuatCmd & msg)
  : msg_(msg)
  {}
  Init_GimbalAngleQuatCmd_orientation vehicle_name(::airsim_interfaces::msg::GimbalAngleQuatCmd::_vehicle_name_type arg)
  {
    msg_.vehicle_name = std::move(arg);
    return Init_GimbalAngleQuatCmd_orientation(msg_);
  }

private:
  ::airsim_interfaces::msg::GimbalAngleQuatCmd msg_;
};

class Init_GimbalAngleQuatCmd_camera_name
{
public:
  explicit Init_GimbalAngleQuatCmd_camera_name(::airsim_interfaces::msg::GimbalAngleQuatCmd & msg)
  : msg_(msg)
  {}
  Init_GimbalAngleQuatCmd_vehicle_name camera_name(::airsim_interfaces::msg::GimbalAngleQuatCmd::_camera_name_type arg)
  {
    msg_.camera_name = std::move(arg);
    return Init_GimbalAngleQuatCmd_vehicle_name(msg_);
  }

private:
  ::airsim_interfaces::msg::GimbalAngleQuatCmd msg_;
};

class Init_GimbalAngleQuatCmd_header
{
public:
  Init_GimbalAngleQuatCmd_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalAngleQuatCmd_camera_name header(::airsim_interfaces::msg::GimbalAngleQuatCmd::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GimbalAngleQuatCmd_camera_name(msg_);
  }

private:
  ::airsim_interfaces::msg::GimbalAngleQuatCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::msg::GimbalAngleQuatCmd>()
{
  return airsim_interfaces::msg::builder::Init_GimbalAngleQuatCmd_header();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_QUAT_CMD__BUILDER_HPP_
