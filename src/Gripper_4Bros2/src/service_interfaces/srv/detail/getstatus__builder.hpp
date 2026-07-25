// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from service_interfaces:srv/Getstatus.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_INTERFACES__SRV__DETAIL__GETSTATUS__BUILDER_HPP_
#define SERVICE_INTERFACES__SRV__DETAIL__GETSTATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "service_interfaces/srv/detail/getstatus__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace service_interfaces
{

namespace srv
{

namespace builder
{

class Init_Getstatus_Request_gripper_id
{
public:
  explicit Init_Getstatus_Request_gripper_id(::service_interfaces::srv::Getstatus_Request & msg)
  : msg_(msg)
  {}
  ::service_interfaces::srv::Getstatus_Request gripper_id(::service_interfaces::srv::Getstatus_Request::_gripper_id_type arg)
  {
    msg_.gripper_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_interfaces::srv::Getstatus_Request msg_;
};

class Init_Getstatus_Request_status
{
public:
  Init_Getstatus_Request_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Getstatus_Request_gripper_id status(::service_interfaces::srv::Getstatus_Request::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Getstatus_Request_gripper_id(msg_);
  }

private:
  ::service_interfaces::srv::Getstatus_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_interfaces::srv::Getstatus_Request>()
{
  return service_interfaces::srv::builder::Init_Getstatus_Request_status();
}

}  // namespace service_interfaces


namespace service_interfaces
{

namespace srv
{

namespace builder
{

class Init_Getstatus_Response_temp
{
public:
  explicit Init_Getstatus_Response_temp(::service_interfaces::srv::Getstatus_Response & msg)
  : msg_(msg)
  {}
  ::service_interfaces::srv::Getstatus_Response temp(::service_interfaces::srv::Getstatus_Response::_temp_type arg)
  {
    msg_.temp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_interfaces::srv::Getstatus_Response msg_;
};

class Init_Getstatus_Response_error
{
public:
  explicit Init_Getstatus_Response_error(::service_interfaces::srv::Getstatus_Response & msg)
  : msg_(msg)
  {}
  Init_Getstatus_Response_temp error(::service_interfaces::srv::Getstatus_Response::_error_type arg)
  {
    msg_.error = std::move(arg);
    return Init_Getstatus_Response_temp(msg_);
  }

private:
  ::service_interfaces::srv::Getstatus_Response msg_;
};

class Init_Getstatus_Response_status
{
public:
  Init_Getstatus_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Getstatus_Response_error status(::service_interfaces::srv::Getstatus_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Getstatus_Response_error(msg_);
  }

private:
  ::service_interfaces::srv::Getstatus_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_interfaces::srv::Getstatus_Response>()
{
  return service_interfaces::srv::builder::Init_Getstatus_Response_status();
}

}  // namespace service_interfaces

#endif  // SERVICE_INTERFACES__SRV__DETAIL__GETSTATUS__BUILDER_HPP_
