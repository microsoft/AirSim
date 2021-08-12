// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from airsim_interfaces:srv/Land.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__LAND__STRUCT_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__LAND__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__srv__Land_Request __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__srv__Land_Request __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Land_Request_
{
  using Type = Land_Request_<ContainerAllocator>;

  explicit Land_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->wait_on_last_task = false;
    }
  }

  explicit Land_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->wait_on_last_task = false;
    }
  }

  // field types and members
  using _wait_on_last_task_type =
    bool;
  _wait_on_last_task_type wait_on_last_task;

  // setters for named parameter idiom
  Type & set__wait_on_last_task(
    const bool & _arg)
  {
    this->wait_on_last_task = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    airsim_interfaces::srv::Land_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::srv::Land_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::srv::Land_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::srv::Land_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::Land_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::Land_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::Land_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::Land_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::srv::Land_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::srv::Land_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__srv__Land_Request
    std::shared_ptr<airsim_interfaces::srv::Land_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__srv__Land_Request
    std::shared_ptr<airsim_interfaces::srv::Land_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Land_Request_ & other) const
  {
    if (this->wait_on_last_task != other.wait_on_last_task) {
      return false;
    }
    return true;
  }
  bool operator!=(const Land_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Land_Request_

// alias to use template instance with default allocator
using Land_Request =
  airsim_interfaces::srv::Land_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace airsim_interfaces


#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__srv__Land_Response __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__srv__Land_Response __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Land_Response_
{
  using Type = Land_Response_<ContainerAllocator>;

  explicit Land_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit Land_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    airsim_interfaces::srv::Land_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::srv::Land_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::srv::Land_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::srv::Land_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::Land_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::Land_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::Land_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::Land_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::srv::Land_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::srv::Land_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__srv__Land_Response
    std::shared_ptr<airsim_interfaces::srv::Land_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__srv__Land_Response
    std::shared_ptr<airsim_interfaces::srv::Land_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Land_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const Land_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Land_Response_

// alias to use template instance with default allocator
using Land_Response =
  airsim_interfaces::srv::Land_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace airsim_interfaces

namespace airsim_interfaces
{

namespace srv
{

struct Land
{
  using Request = airsim_interfaces::srv::Land_Request;
  using Response = airsim_interfaces::srv::Land_Response;
};

}  // namespace srv

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__LAND__STRUCT_HPP_
