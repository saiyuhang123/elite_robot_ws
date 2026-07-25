// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from service_interfaces:srv/Setopenlimit.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_INTERFACES__SRV__DETAIL__SETOPENLIMIT__BUILDER_HPP_
#define SERVICE_INTERFACES__SRV__DETAIL__SETOPENLIMIT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "service_interfaces/srv/detail/setopenlimit__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace service_interfaces
{

namespace srv
{

namespace builder
{

class Init_Setopenlimit_Request_openmin
{
public:
  explicit Init_Setopenlimit_Request_openmin(::service_interfaces::srv::Setopenlimit_Request & msg)
  : msg_(msg)
  {}
  ::service_interfaces::srv::Setopenlimit_Request openmin(::service_interfaces::srv::Setopenlimit_Request::_openmin_type arg)
  {
    msg_.openmin = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_interfaces::srv::Setopenlimit_Request msg_;
};

class Init_Setopenlimit_Request_openmax
{
public:
  explicit Init_Setopenlimit_Request_openmax(::service_interfaces::srv::Setopenlimit_Request & msg)
  : msg_(msg)
  {}
  Init_Setopenlimit_Request_openmin openmax(::service_interfaces::srv::Setopenlimit_Request::_openmax_type arg)
  {
    msg_.openmax = std::move(arg);
    return Init_Setopenlimit_Request_openmin(msg_);
  }

private:
  ::service_interfaces::srv::Setopenlimit_Request msg_;
};

class Init_Setopenlimit_Request_gripper_id
{
public:
  explicit Init_Setopenlimit_Request_gripper_id(::service_interfaces::srv::Setopenlimit_Request & msg)
  : msg_(msg)
  {}
  Init_Setopenlimit_Request_openmax gripper_id(::service_interfaces::srv::Setopenlimit_Request::_gripper_id_type arg)
  {
    msg_.gripper_id = std::move(arg);
    return Init_Setopenlimit_Request_openmax(msg_);
  }

private:
  ::service_interfaces::srv::Setopenlimit_Request msg_;
};

class Init_Setopenlimit_Request_status
{
public:
  Init_Setopenlimit_Request_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Setopenlimit_Request_gripper_id status(::service_interfaces::srv::Setopenlimit_Request::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Setopenlimit_Request_gripper_id(msg_);
  }

private:
  ::service_interfaces::srv::Setopenlimit_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_interfaces::srv::Setopenlimit_Request>()
{
  return service_interfaces::srv::builder::Init_Setopenlimit_Request_status();
}

}  // namespace service_interfaces


namespace service_interfaces
{

namespace srv
{

namespace builder
{

class Init_Setopenlimit_Response_openlimit_accepted
{
public:
  Init_Setopenlimit_Response_openlimit_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::service_interfaces::srv::Setopenlimit_Response openlimit_accepted(::service_interfaces::srv::Setopenlimit_Response::_openlimit_accepted_type arg)
  {
    msg_.openlimit_accepted = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_interfaces::srv::Setopenlimit_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_interfaces::srv::Setopenlimit_Response>()
{
  return service_interfaces::srv::builder::Init_Setopenlimit_Response_openlimit_accepted();
}

}  // namespace service_interfaces

#endif  // SERVICE_INTERFACES__SRV__DETAIL__SETOPENLIMIT__BUILDER_HPP_
