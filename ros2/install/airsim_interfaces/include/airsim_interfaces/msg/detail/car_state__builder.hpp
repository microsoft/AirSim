// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:msg/CarState.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__CAR_STATE__BUILDER_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__CAR_STATE__BUILDER_HPP_

#include "airsim_interfaces/msg/detail/car_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace airsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_CarState_handbrake
{
public:
  explicit Init_CarState_handbrake(::airsim_interfaces::msg::CarState & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::msg::CarState handbrake(::airsim_interfaces::msg::CarState::_handbrake_type arg)
  {
    msg_.handbrake = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::msg::CarState msg_;
};

class Init_CarState_maxrpm
{
public:
  explicit Init_CarState_maxrpm(::airsim_interfaces::msg::CarState & msg)
  : msg_(msg)
  {}
  Init_CarState_handbrake maxrpm(::airsim_interfaces::msg::CarState::_maxrpm_type arg)
  {
    msg_.maxrpm = std::move(arg);
    return Init_CarState_handbrake(msg_);
  }

private:
  ::airsim_interfaces::msg::CarState msg_;
};

class Init_CarState_rpm
{
public:
  explicit Init_CarState_rpm(::airsim_interfaces::msg::CarState & msg)
  : msg_(msg)
  {}
  Init_CarState_maxrpm rpm(::airsim_interfaces::msg::CarState::_rpm_type arg)
  {
    msg_.rpm = std::move(arg);
    return Init_CarState_maxrpm(msg_);
  }

private:
  ::airsim_interfaces::msg::CarState msg_;
};

class Init_CarState_gear
{
public:
  explicit Init_CarState_gear(::airsim_interfaces::msg::CarState & msg)
  : msg_(msg)
  {}
  Init_CarState_rpm gear(::airsim_interfaces::msg::CarState::_gear_type arg)
  {
    msg_.gear = std::move(arg);
    return Init_CarState_rpm(msg_);
  }

private:
  ::airsim_interfaces::msg::CarState msg_;
};

class Init_CarState_speed
{
public:
  explicit Init_CarState_speed(::airsim_interfaces::msg::CarState & msg)
  : msg_(msg)
  {}
  Init_CarState_gear speed(::airsim_interfaces::msg::CarState::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_CarState_gear(msg_);
  }

private:
  ::airsim_interfaces::msg::CarState msg_;
};

class Init_CarState_twist
{
public:
  explicit Init_CarState_twist(::airsim_interfaces::msg::CarState & msg)
  : msg_(msg)
  {}
  Init_CarState_speed twist(::airsim_interfaces::msg::CarState::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return Init_CarState_speed(msg_);
  }

private:
  ::airsim_interfaces::msg::CarState msg_;
};

class Init_CarState_pose
{
public:
  explicit Init_CarState_pose(::airsim_interfaces::msg::CarState & msg)
  : msg_(msg)
  {}
  Init_CarState_twist pose(::airsim_interfaces::msg::CarState::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_CarState_twist(msg_);
  }

private:
  ::airsim_interfaces::msg::CarState msg_;
};

class Init_CarState_header
{
public:
  Init_CarState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CarState_pose header(::airsim_interfaces::msg::CarState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CarState_pose(msg_);
  }

private:
  ::airsim_interfaces::msg::CarState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::msg::CarState>()
{
  return airsim_interfaces::msg::builder::Init_CarState_header();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__CAR_STATE__BUILDER_HPP_
