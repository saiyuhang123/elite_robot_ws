// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from service_interfaces:srv/Setclearerror.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_INTERFACES__SRV__DETAIL__SETCLEARERROR__BUILDER_HPP_
#define SERVICE_INTERFACES__SRV__DETAIL__SETCLEARERROR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "service_interfaces/srv/detail/setclearerror__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace service_interfaces
{

namespace srv
{

namespace builder
{

class Init_Setclearerror_Request_gripper_id
{
public:
  explicit Init_Setclearerror_Request_gripper_id(::service_interfaces::srv::Setclearerror_Request & msg)
  : msg_(msg)
  {}
  ::service_interfaces::srv::Setclearerror_Request gripper_id(::service_interfaces::srv::Setclearerror_Request::_gripper_id_type arg)
  {
    msg_.gripper_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_interfaces::srv::Setclearerror_Request msg_;
};

class Init_Setclearerror_Request_status
{
public:
  Init_Setclearerror_Request_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Setclearerror_Request_gripper_id status(::service_interfaces::srv::Setclearerror_Request::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Setclearerror_Request_gripper_id(msg_);
  }

private:
  ::service_interfaces::srv::Setclearerror_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_interfaces::srv::Setclearerror_Request>()
{
  return service_interfaces::srv::builder::Init_Setclearerror_Request_status();
}

}  // namespace service_interfaces


namespace service_interfaces
{

namespace srv
{

namespace builder
{

class Init_Setclearerror_Response_clearerror_accepted
{
public:
  Init_Setclearerror_Response_clearerror_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::service_interfaces::srv::Setclearerror_Response clearerror_accepted(::service_interfaces::srv::Setclearerror_Response::_clearerror_accepted_type arg)
  {
    msg_.clearerror_accepted = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_interfaces::srv::Setclearerror_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_interfaces::srv::Setclearerror_Response>()
{
  return service_interfaces::srv::builder::Init_Setclearerror_Response_clearerror_accepted();
}

}  // namespace service_interfaces

#endif  // SERVICE_INTERFACES__SRV__DETAIL__SETCLEARERROR__BUILDER_HPP_
