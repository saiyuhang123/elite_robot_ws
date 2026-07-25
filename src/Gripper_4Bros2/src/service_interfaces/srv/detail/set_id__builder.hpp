// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from service_interfaces:srv/SetID.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_INTERFACES__SRV__DETAIL__SET_ID__BUILDER_HPP_
#define SERVICE_INTERFACES__SRV__DETAIL__SET_ID__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "service_interfaces/srv/detail/set_id__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace service_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetID_Request_gripper_setid
{
public:
  explicit Init_SetID_Request_gripper_setid(::service_interfaces::srv::SetID_Request & msg)
  : msg_(msg)
  {}
  ::service_interfaces::srv::SetID_Request gripper_setid(::service_interfaces::srv::SetID_Request::_gripper_setid_type arg)
  {
    msg_.gripper_setid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_interfaces::srv::SetID_Request msg_;
};

class Init_SetID_Request_gripper_id
{
public:
  explicit Init_SetID_Request_gripper_id(::service_interfaces::srv::SetID_Request & msg)
  : msg_(msg)
  {}
  Init_SetID_Request_gripper_setid gripper_id(::service_interfaces::srv::SetID_Request::_gripper_id_type arg)
  {
    msg_.gripper_id = std::move(arg);
    return Init_SetID_Request_gripper_setid(msg_);
  }

private:
  ::service_interfaces::srv::SetID_Request msg_;
};

class Init_SetID_Request_status
{
public:
  Init_SetID_Request_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetID_Request_gripper_id status(::service_interfaces::srv::SetID_Request::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_SetID_Request_gripper_id(msg_);
  }

private:
  ::service_interfaces::srv::SetID_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_interfaces::srv::SetID_Request>()
{
  return service_interfaces::srv::builder::Init_SetID_Request_status();
}

}  // namespace service_interfaces


namespace service_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetID_Response_id_accepted
{
public:
  Init_SetID_Response_id_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::service_interfaces::srv::SetID_Response id_accepted(::service_interfaces::srv::SetID_Response::_id_accepted_type arg)
  {
    msg_.id_accepted = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_interfaces::srv::SetID_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_interfaces::srv::SetID_Response>()
{
  return service_interfaces::srv::builder::Init_SetID_Response_id_accepted();
}

}  // namespace service_interfaces

#endif  // SERVICE_INTERFACES__SRV__DETAIL__SET_ID__BUILDER_HPP_
