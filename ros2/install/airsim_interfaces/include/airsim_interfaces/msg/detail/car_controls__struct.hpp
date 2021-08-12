// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from airsim_interfaces:msg/CarControls.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__CAR_CONTROLS__STRUCT_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__CAR_CONTROLS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__msg__CarControls __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__msg__CarControls __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CarControls_
{
  using Type = CarControls_<ContainerAllocator>;

  explicit CarControls_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->throttle = 0.0f;
      this->brake = 0.0f;
      this->steering = 0.0f;
      this->handbrake = false;
      this->manual = false;
      this->manual_gear = 0;
      this->gear_immediate = false;
    }
  }

  explicit CarControls_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->throttle = 0.0f;
      this->brake = 0.0f;
      this->steering = 0.0f;
      this->handbrake = false;
      this->manual = false;
      this->manual_gear = 0;
      this->gear_immediate = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _throttle_type =
    float;
  _throttle_type throttle;
  using _brake_type =
    float;
  _brake_type brake;
  using _steering_type =
    float;
  _steering_type steering;
  using _handbrake_type =
    bool;
  _handbrake_type handbrake;
  using _manual_type =
    bool;
  _manual_type manual;
  using _manual_gear_type =
    int8_t;
  _manual_gear_type manual_gear;
  using _gear_immediate_type =
    bool;
  _gear_immediate_type gear_immediate;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__throttle(
    const float & _arg)
  {
    this->throttle = _arg;
    return *this;
  }
  Type & set__brake(
    const float & _arg)
  {
    this->brake = _arg;
    return *this;
  }
  Type & set__steering(
    const float & _arg)
  {
    this->steering = _arg;
    return *this;
  }
  Type & set__handbrake(
    const bool & _arg)
  {
    this->handbrake = _arg;
    return *this;
  }
  Type & set__manual(
    const bool & _arg)
  {
    this->manual = _arg;
    return *this;
  }
  Type & set__manual_gear(
    const int8_t & _arg)
  {
    this->manual_gear = _arg;
    return *this;
  }
  Type & set__gear_immediate(
    const bool & _arg)
  {
    this->gear_immediate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    airsim_interfaces::msg::CarControls_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::msg::CarControls_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::msg::CarControls_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::msg::CarControls_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::CarControls_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::CarControls_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::CarControls_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::CarControls_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::msg::CarControls_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::msg::CarControls_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__msg__CarControls
    std::shared_ptr<airsim_interfaces::msg::CarControls_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__msg__CarControls
    std::shared_ptr<airsim_interfaces::msg::CarControls_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CarControls_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->throttle != other.throttle) {
      return false;
    }
    if (this->brake != other.brake) {
      return false;
    }
    if (this->steering != other.steering) {
      return false;
    }
    if (this->handbrake != other.handbrake) {
      return false;
    }
    if (this->manual != other.manual) {
      return false;
    }
    if (this->manual_gear != other.manual_gear) {
      return false;
    }
    if (this->gear_immediate != other.gear_immediate) {
      return false;
    }
    return true;
  }
  bool operator!=(const CarControls_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CarControls_

// alias to use template instance with default allocator
using CarControls =
  airsim_interfaces::msg::CarControls_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__CAR_CONTROLS__STRUCT_HPP_
