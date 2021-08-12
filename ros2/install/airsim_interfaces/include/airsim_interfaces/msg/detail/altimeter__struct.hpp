// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from airsim_interfaces:msg/Altimeter.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__ALTIMETER__STRUCT_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__ALTIMETER__STRUCT_HPP_

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
# define DEPRECATED__airsim_interfaces__msg__Altimeter __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__msg__Altimeter __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Altimeter_
{
  using Type = Altimeter_<ContainerAllocator>;

  explicit Altimeter_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->altitude = 0.0f;
      this->pressure = 0.0f;
      this->qnh = 0.0f;
    }
  }

  explicit Altimeter_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->altitude = 0.0f;
      this->pressure = 0.0f;
      this->qnh = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _altitude_type =
    float;
  _altitude_type altitude;
  using _pressure_type =
    float;
  _pressure_type pressure;
  using _qnh_type =
    float;
  _qnh_type qnh;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__altitude(
    const float & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__pressure(
    const float & _arg)
  {
    this->pressure = _arg;
    return *this;
  }
  Type & set__qnh(
    const float & _arg)
  {
    this->qnh = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    airsim_interfaces::msg::Altimeter_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::msg::Altimeter_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::msg::Altimeter_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::msg::Altimeter_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::Altimeter_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::Altimeter_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::Altimeter_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::Altimeter_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::msg::Altimeter_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::msg::Altimeter_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__msg__Altimeter
    std::shared_ptr<airsim_interfaces::msg::Altimeter_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__msg__Altimeter
    std::shared_ptr<airsim_interfaces::msg::Altimeter_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Altimeter_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->pressure != other.pressure) {
      return false;
    }
    if (this->qnh != other.qnh) {
      return false;
    }
    return true;
  }
  bool operator!=(const Altimeter_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Altimeter_

// alias to use template instance with default allocator
using Altimeter =
  airsim_interfaces::msg::Altimeter_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__ALTIMETER__STRUCT_HPP_
