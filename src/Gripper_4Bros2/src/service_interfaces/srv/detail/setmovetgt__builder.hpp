// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from service_interfaces:srv/Setmovetgt.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_INTERFACES__SRV__DETAIL__SETMOVETGT__BUILDER_HPP_
#define SERVICE_INTERFACES__SRV__DETAIL__SETMOVETGT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "service_interfaces/srv/detail/setmovetgt__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace service_interfaces
{

namespace srv
{

namespace builder
{

class Init_Setmovetgt_Request_movetgt
{
public:
  explicit Init_Setmovetgt_Request_movetgt(::service_interfaces::srv::Setmovetgt_Request & msg)
  : msg_(msg)
  {}
  ::service_interfaces::srv::Setmovetgt_Request movetgt(::service_interfaces::srv::Setmovetgt_Request::_movetgt_type arg)
  {
    msg_.movetgt = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_interfaces::srv::Setmovetgt_Request msg_;
};

class Init_Setmovetgt_Request_gripper_id
{
public:
  explicit Init_Setmovetgt_Request_gripper_id(::service_interfaces::srv::Setmovetgt_Request & msg)
  : msg_(msg)
  {}
  Init_Setmovetgt_Request_movetgt gripper_id(::service_interfaces::srv::Setmovetgt_Request::_gripper_id_type arg)
  {
    msg_.gripper_id = std::move(arg);
    return Init_Setmovetgt_Request_movetgt(msg_);
  }

private:
  ::service_interfaces::srv::Setmovetgt_Request msg_;
};

class Init_Setmovetgt_Request_status
{
public:
  Init_Setmovetgt_Request_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Setmovetgt_Request_gripper_id status(::service_interfaces::srv::Setmovetgt_Request::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Setmovetgt_Request_gripper_id(msg_);
  }

private:
  ::service_interfaces::srv::Setmovetgt_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_interfaces::srv::Setmovetgt_Request>()
{
  return service_interfaces::srv::builder::Init_Setmovetgt_Request_status();
}

}  // namespace service_interfaces


namespace service_interfaces
{

namespace srv
{

namespace builder
{

class Init_Setmovetgt_Response_movetgt_accepted
{
public:
  Init_Setmovetgt_Response_movetgt_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::service_interfaces::srv::Setmovetgt_Response movetgt_accepted(::service_interfaces::srv::Setmovetgt_Response::_movetgt_accepted_type arg)
  {
    msg_.movetgt_accepted = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_interfaces::srv::Setmovetgt_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_interfaces::srv::Setmovetgt_Response>()
{
  return service_interfaces::srv::builder::Init_Setmovetgt_Response_movetgt_accepted();
}

}  // namespace service_interfaces

#endif  // SERVICE_INTERFACES__SRV__DETAIL__SETMOVETGT__BUILDER_HPP_
