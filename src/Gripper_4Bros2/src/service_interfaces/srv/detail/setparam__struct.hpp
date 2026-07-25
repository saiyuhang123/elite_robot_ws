// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from service_interfaces:srv/Setparam.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_INTERFACES__SRV__DETAIL__SETPARAM__STRUCT_HPP_
#define SERVICE_INTERFACES__SRV__DETAIL__SETPARAM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__service_interfaces__srv__Setparam_Request __attribute__((deprecated))
#else
# define DEPRECATED__service_interfaces__srv__Setparam_Request __declspec(deprecated)
#endif

namespace service_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Setparam_Request_
{
  using Type = Setparam_Request_<ContainerAllocator>;

  explicit Setparam_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = "";
      this->gripper_id = 0l;
    }
  }

  explicit Setparam_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = "";
      this->gripper_id = 0l;
    }
  }

  // field types and members
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;
  using _gripper_id_type =
    int32_t;
  _gripper_id_type gripper_id;

  // setters for named parameter idiom
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__gripper_id(
    const int32_t & _arg)
  {
    this->gripper_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    service_interfaces::srv::Setparam_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const service_interfaces::srv::Setparam_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<service_interfaces::srv::Setparam_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<service_interfaces::srv::Setparam_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      service_interfaces::srv::Setparam_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<service_interfaces::srv::Setparam_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      service_interfaces::srv::Setparam_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<service_interfaces::srv::Setparam_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<service_interfaces::srv::Setparam_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<service_interfaces::srv::Setparam_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__service_interfaces__srv__Setparam_Request
    std::shared_ptr<service_interfaces::srv::Setparam_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__service_interfaces__srv__Setparam_Request
    std::shared_ptr<service_interfaces::srv::Setparam_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Setparam_Request_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->gripper_id != other.gripper_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Setparam_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Setparam_Request_

// alias to use template instance with default allocator
using Setparam_Request =
  service_interfaces::srv::Setparam_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace service_interfaces


#ifndef _WIN32
# define DEPRECATED__service_interfaces__srv__Setparam_Response __attribute__((deprecated))
#else
# define DEPRECATED__service_interfaces__srv__Setparam_Response __declspec(deprecated)
#endif

namespace service_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Setparam_Response_
{
  using Type = Setparam_Response_<ContainerAllocator>;

  explicit Setparam_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->param_accepted = false;
    }
  }

  explicit Setparam_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->param_accepted = false;
    }
  }

  // field types and members
  using _param_accepted_type =
    bool;
  _param_accepted_type param_accepted;

  // setters for named parameter idiom
  Type & set__param_accepted(
    const bool & _arg)
  {
    this->param_accepted = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    service_interfaces::srv::Setparam_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const service_interfaces::srv::Setparam_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<service_interfaces::srv::Setparam_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<service_interfaces::srv::Setparam_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      service_interfaces::srv::Setparam_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<service_interfaces::srv::Setparam_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      service_interfaces::srv::Setparam_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<service_interfaces::srv::Setparam_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<service_interfaces::srv::Setparam_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<service_interfaces::srv::Setparam_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__service_interfaces__srv__Setparam_Response
    std::shared_ptr<service_interfaces::srv::Setparam_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__service_interfaces__srv__Setparam_Response
    std::shared_ptr<service_interfaces::srv::Setparam_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Setparam_Response_ & other) const
  {
    if (this->param_accepted != other.param_accepted) {
      return false;
    }
    return true;
  }
  bool operator!=(const Setparam_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Setparam_Response_

// alias to use template instance with default allocator
using Setparam_Response =
  service_interfaces::srv::Setparam_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace service_interfaces

namespace service_interfaces
{

namespace srv
{

struct Setparam
{
  using Request = service_interfaces::srv::Setparam_Request;
  using Response = service_interfaces::srv::Setparam_Response;
};

}  // namespace srv

}  // namespace service_interfaces

#endif  // SERVICE_INTERFACES__SRV__DETAIL__SETPARAM__STRUCT_HPP_
