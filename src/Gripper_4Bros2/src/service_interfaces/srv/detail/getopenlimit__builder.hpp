// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from service_interfaces:srv/Getopenlimit.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_INTERFACES__SRV__DETAIL__GETOPENLIMIT__BUILDER_HPP_
#define SERVICE_INTERFACES__SRV__DETAIL__GETOPENLIMIT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "service_interfaces/srv/detail/getopenlimit__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace service_interfaces
{

namespace srv
{

namespace builder
{

class Init_Getopenlimit_Request_gripper_id
{
public:
  explicit Init_Getopenlimit_Request_gripper_id(::service_interfaces::srv::Getopenlimit_Request & msg)
  : msg_(msg)
  {}
  ::service_interfaces::srv::Getopenlimit_Request gripper_id(::service_interfaces::srv::Getopenlimit_Request::_gripper_id_type arg)
  {
    msg_.gripper_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_interfaces::srv::Getopenlimit_Request msg_;
};

class Init_Getopenlimit_Request_status
{
public:
  Init_Getopenlimit_Request_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Getopenlimit_Request_gripper_id status(::service_interfaces::srv::Getopenlimit_Request::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Getopenlimit_Request_gripper_id(msg_);
  }

private:
  ::service_interfaces::srv::Getopenlimit_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_interfaces::srv::Getopenlimit_Request>()
{
  return service_interfaces::srv::builder::Init_Getopenlimit_Request_status();
}

}  // namespace service_interfaces


namespace service_interfaces
{

namespace srv
{

namespace builder
{

class Init_Getopenlimit_Response_openmin
{
public:
  explicit Init_Getopenlimit_Response_openmin(::service_interfaces::srv::Getopenlimit_Response & msg)
  : msg_(msg)
  {}
  ::service_interfaces::srv::Getopenlimit_Response openmin(::service_interfaces::srv::Getopenlimit_Response::_openmin_type arg)
  {
    msg_.openmin = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_interfaces::srv::Getopenlimit_Response msg_;
};

class Init_Getopenlimit_Response_openmax
{
public:
  Init_Getopenlimit_Response_openmax()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Getopenlimit_Response_openmin openmax(::service_interfaces::srv::Getopenlimit_Response::_openmax_type arg)
  {
    msg_.openmax = std::move(arg);
    return Init_Getopenlimit_Response_openmin(msg_);
  }

private:
  ::service_interfaces::srv::Getopenlimit_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_interfaces::srv::Getopenlimit_Response>()
{
  return service_interfaces::srv::builder::Init_Getopenlimit_Response_openmax();
}

}  // namespace service_interfaces

#endif  // SERVICE_INTERFACES__SRV__DETAIL__GETOPENLIMIT__BUILDER_HPP_
