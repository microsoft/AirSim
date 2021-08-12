// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from airsim_interfaces:srv/TakeoffGroup.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__TAKEOFF_GROUP__STRUCT_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__TAKEOFF_GROUP__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__srv__TakeoffGroup_Request __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__srv__TakeoffGroup_Request __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TakeoffGroup_Request_
{
  using Type = TakeoffGroup_Request_<ContainerAllocator>;

  explicit TakeoffGroup_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->wait_on_last_task = false;
    }
  }

  explicit TakeoffGroup_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->wait_on_last_task = false;
    }
  }

  // field types and members
  using _vehicle_names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _vehicle_names_type vehicle_names;
  using _wait_on_last_task_type =
    bool;
  _wait_on_last_task_type wait_on_last_task;

  // setters for named parameter idiom
  Type & set__vehicle_names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->vehicle_names = _arg;
    return *this;
  }
  Type & set__wait_on_last_task(
    const bool & _arg)
  {
    this->wait_on_last_task = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    airsim_interfaces::srv::TakeoffGroup_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::srv::TakeoffGroup_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::srv::TakeoffGroup_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::srv::TakeoffGroup_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::TakeoffGroup_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::TakeoffGroup_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::TakeoffGroup_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::TakeoffGroup_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::srv::TakeoffGroup_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::srv::TakeoffGroup_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__srv__TakeoffGroup_Request
    std::shared_ptr<airsim_interfaces::srv::TakeoffGroup_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__srv__TakeoffGroup_Request
    std::shared_ptr<airsim_interfaces::srv::TakeoffGroup_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TakeoffGroup_Request_ & other) const
  {
    if (this->vehicle_names != other.vehicle_names) {
      return false;
    }
    if (this->wait_on_last_task != other.wait_on_last_task) {
      return false;
    }
    return true;
  }
  bool operator!=(const TakeoffGroup_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TakeoffGroup_Request_

// alias to use template instance with default allocator
using TakeoffGroup_Request =
  airsim_interfaces::srv::TakeoffGroup_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace airsim_interfaces


#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__srv__TakeoffGroup_Response __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__srv__TakeoffGroup_Response __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TakeoffGroup_Response_
{
  using Type = TakeoffGroup_Response_<ContainerAllocator>;

  explicit TakeoffGroup_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit TakeoffGroup_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    airsim_interfaces::srv::TakeoffGroup_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::srv::TakeoffGroup_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::srv::TakeoffGroup_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::srv::TakeoffGroup_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::TakeoffGroup_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::TakeoffGroup_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::TakeoffGroup_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::TakeoffGroup_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::srv::TakeoffGroup_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::srv::TakeoffGroup_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__srv__TakeoffGroup_Response
    std::shared_ptr<airsim_interfaces::srv::TakeoffGroup_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__srv__TakeoffGroup_Response
    std::shared_ptr<airsim_interfaces::srv::TakeoffGroup_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TakeoffGroup_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const TakeoffGroup_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TakeoffGroup_Response_

// alias to use template instance with default allocator
using TakeoffGroup_Response =
  airsim_interfaces::srv::TakeoffGroup_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace airsim_interfaces

namespace airsim_interfaces
{

namespace srv
{

struct TakeoffGroup
{
  using Request = airsim_interfaces::srv::TakeoffGroup_Request;
  using Response = airsim_interfaces::srv::TakeoffGroup_Response;
};

}  // namespace srv

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__TAKEOFF_GROUP__STRUCT_HPP_
