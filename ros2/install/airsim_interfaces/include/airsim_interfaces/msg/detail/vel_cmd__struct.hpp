// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from airsim_interfaces:msg/VelCmd.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__STRUCT_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__msg__VelCmd __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__msg__VelCmd __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VelCmd_
{
  using Type = VelCmd_<ContainerAllocator>;

  explicit VelCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : twist(_init)
  {
    (void)_init;
  }

  explicit VelCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : twist(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _twist_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _twist_type twist;

  // setters for named parameter idiom
  Type & set__twist(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->twist = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    airsim_interfaces::msg::VelCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::msg::VelCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::msg::VelCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::msg::VelCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::VelCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::VelCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::VelCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::VelCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::msg::VelCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::msg::VelCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__msg__VelCmd
    std::shared_ptr<airsim_interfaces::msg::VelCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__msg__VelCmd
    std::shared_ptr<airsim_interfaces::msg::VelCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VelCmd_ & other) const
  {
    if (this->twist != other.twist) {
      return false;
    }
    return true;
  }
  bool operator!=(const VelCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VelCmd_

// alias to use template instance with default allocator
using VelCmd =
  airsim_interfaces::msg::VelCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__STRUCT_HPP_
