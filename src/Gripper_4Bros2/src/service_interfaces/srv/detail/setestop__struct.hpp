// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from service_interfaces:srv/Setestop.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_INTERFACES__SRV__DETAIL__SETESTOP__STRUCT_HPP_
#define SERVICE_INTERFACES__SRV__DETAIL__SETESTOP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__service_interfaces__srv__Setestop_Request __attribute__((deprecated))
#else
# define DEPRECATED__service_interfaces__srv__Setestop_Request __declspec(deprecated)
#endif

namespace service_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Setestop_Request_
{
  using Type = Setestop_Request_<ContainerAllocator>;

  explicit Setestop_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = "";
      this->gripper_id = 0l;
    }
  }

  explicit Setestop_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    service_interfaces::srv::Setestop_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const service_interfaces::srv::Setestop_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<service_interfaces::srv::Setestop_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<service_interfaces::srv::Setestop_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      service_interfaces::srv::Setestop_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<service_interfaces::srv::Setestop_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      service_interfaces::srv::Setestop_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<service_interfaces::srv::Setestop_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<service_interfaces::srv::Setestop_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<service_interfaces::srv::Setestop_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__service_interfaces__srv__Setestop_Request
    std::shared_ptr<service_interfaces::srv::Setestop_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__service_interfaces__srv__Setestop_Request
    std::shared_ptr<service_interfaces::srv::Setestop_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Setestop_Request_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->gripper_id != other.gripper_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Setestop_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Setestop_Request_

// alias to use template instance with default allocator
using Setestop_Request =
  service_interfaces::srv::Setestop_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace service_interfaces


#ifndef _WIN32
# define DEPRECATED__service_interfaces__srv__Setestop_Response __attribute__((deprecated))
#else
# define DEPRECATED__service_interfaces__srv__Setestop_Response __declspec(deprecated)
#endif

namespace service_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Setestop_Response_
{
  using Type = Setestop_Response_<ContainerAllocator>;

  explicit Setestop_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->estop_accepted = false;
    }
  }

  explicit Setestop_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->estop_accepted = false;
    }
  }

  // field types and members
  using _estop_accepted_type =
    bool;
  _estop_accepted_type estop_accepted;

  // setters for named parameter idiom
  Type & set__estop_accepted(
    const bool & _arg)
  {
    this->estop_accepted = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    service_interfaces::srv::Setestop_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const service_interfaces::srv::Setestop_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<service_interfaces::srv::Setestop_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<service_interfaces::srv::Setestop_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      service_interfaces::srv::Setestop_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<service_interfaces::srv::Setestop_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      service_interfaces::srv::Setestop_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<service_interfaces::srv::Setestop_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<service_interfaces::srv::Setestop_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<service_interfaces::srv::Setestop_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__service_interfaces__srv__Setestop_Response
    std::shared_ptr<service_interfaces::srv::Setestop_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__service_interfaces__srv__Setestop_Response
    std::shared_ptr<service_interfaces::srv::Setestop_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Setestop_Response_ & other) const
  {
    if (this->estop_accepted != other.estop_accepted) {
      return false;
    }
    return true;
  }
  bool operator!=(const Setestop_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Setestop_Response_

// alias to use template instance with default allocator
using Setestop_Response =
  service_interfaces::srv::Setestop_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace service_interfaces

namespace service_interfaces
{

namespace srv
{

struct Setestop
{
  using Request = service_interfaces::srv::Setestop_Request;
  using Response = service_interfaces::srv::Setestop_Response;
};

}  // namespace srv

}  // namespace service_interfaces

#endif  // SERVICE_INTERFACES__SRV__DETAIL__SETESTOP__STRUCT_HPP_
