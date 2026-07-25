// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from service_interfaces:srv/Getcopen.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_INTERFACES__SRV__DETAIL__GETCOPEN__BUILDER_HPP_
#define SERVICE_INTERFACES__SRV__DETAIL__GETCOPEN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "service_interfaces/srv/detail/getcopen__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace service_interfaces
{

namespace srv
{

namespace builder
{

class Init_Getcopen_Request_gripper_id
{
public:
  explicit Init_Getcopen_Request_gripper_id(::service_interfaces::srv::Getcopen_Request & msg)
  : msg_(msg)
  {}
  ::service_interfaces::srv::Getcopen_Request gripper_id(::service_interfaces::srv::Getcopen_Request::_gripper_id_type arg)
  {
    msg_.gripper_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_interfaces::srv::Getcopen_Request msg_;
};

class Init_Getcopen_Request_status
{
public:
  Init_Getcopen_Request_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Getcopen_Request_gripper_id status(::service_interfaces::srv::Getcopen_Request::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Getcopen_Request_gripper_id(msg_);
  }

private:
  ::service_interfaces::srv::Getcopen_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_interfaces::srv::Getcopen_Request>()
{
  return service_interfaces::srv::builder::Init_Getcopen_Request_status();
}

}  // namespace service_interfaces


namespace service_interfaces
{

namespace srv
{

namespace builder
{

class Init_Getcopen_Response_copen
{
public:
  Init_Getcopen_Response_copen()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::service_interfaces::srv::Getcopen_Response copen(::service_interfaces::srv::Getcopen_Response::_copen_type arg)
  {
    msg_.copen = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_interfaces::srv::Getcopen_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_interfaces::srv::Getcopen_Response>()
{
  return service_interfaces::srv::builder::Init_Getcopen_Response_copen();
}

}  // namespace service_interfaces

#endif  // SERVICE_INTERFACES__SRV__DETAIL__GETCOPEN__BUILDER_HPP_
