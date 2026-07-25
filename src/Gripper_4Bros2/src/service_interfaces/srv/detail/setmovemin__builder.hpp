// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from service_interfaces:srv/Setmovemin.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_INTERFACES__SRV__DETAIL__SETMOVEMIN__BUILDER_HPP_
#define SERVICE_INTERFACES__SRV__DETAIL__SETMOVEMIN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "service_interfaces/srv/detail/setmovemin__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace service_interfaces
{

namespace srv
{

namespace builder
{

class Init_Setmovemin_Request_power
{
public:
  explicit Init_Setmovemin_Request_power(::service_interfaces::srv::Setmovemin_Request & msg)
  : msg_(msg)
  {}
  ::service_interfaces::srv::Setmovemin_Request power(::service_interfaces::srv::Setmovemin_Request::_power_type arg)
  {
    msg_.power = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_interfaces::srv::Setmovemin_Request msg_;
};

class Init_Setmovemin_Request_speed
{
public:
  explicit Init_Setmovemin_Request_speed(::service_interfaces::srv::Setmovemin_Request & msg)
  : msg_(msg)
  {}
  Init_Setmovemin_Request_power speed(::service_interfaces::srv::Setmovemin_Request::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_Setmovemin_Request_power(msg_);
  }

private:
  ::service_interfaces::srv::Setmovemin_Request msg_;
};

class Init_Setmovemin_Request_gripper_id
{
public:
  explicit Init_Setmovemin_Request_gripper_id(::service_interfaces::srv::Setmovemin_Request & msg)
  : msg_(msg)
  {}
  Init_Setmovemin_Request_speed gripper_id(::service_interfaces::srv::Setmovemin_Request::_gripper_id_type arg)
  {
    msg_.gripper_id = std::move(arg);
    return Init_Setmovemin_Request_speed(msg_);
  }

private:
  ::service_interfaces::srv::Setmovemin_Request msg_;
};

class Init_Setmovemin_Request_status
{
public:
  Init_Setmovemin_Request_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Setmovemin_Request_gripper_id status(::service_interfaces::srv::Setmovemin_Request::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Setmovemin_Request_gripper_id(msg_);
  }

private:
  ::service_interfaces::srv::Setmovemin_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_interfaces::srv::Setmovemin_Request>()
{
  return service_interfaces::srv::builder::Init_Setmovemin_Request_status();
}

}  // namespace service_interfaces


namespace service_interfaces
{

namespace srv
{

namespace builder
{

class Init_Setmovemin_Response_movemin_accepted
{
public:
  Init_Setmovemin_Response_movemin_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::service_interfaces::srv::Setmovemin_Response movemin_accepted(::service_interfaces::srv::Setmovemin_Response::_movemin_accepted_type arg)
  {
    msg_.movemin_accepted = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_interfaces::srv::Setmovemin_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_interfaces::srv::Setmovemin_Response>()
{
  return service_interfaces::srv::builder::Init_Setmovemin_Response_movemin_accepted();
}

}  // namespace service_interfaces

#endif  // SERVICE_INTERFACES__SRV__DETAIL__SETMOVEMIN__BUILDER_HPP_
