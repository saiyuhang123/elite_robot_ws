// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from service_interfaces:srv/Setmovemin.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_INTERFACES__SRV__DETAIL__SETMOVEMIN__TRAITS_HPP_
#define SERVICE_INTERFACES__SRV__DETAIL__SETMOVEMIN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "service_interfaces/srv/detail/setmovemin__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace service_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Setmovemin_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: gripper_id
  {
    out << "gripper_id: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper_id, out);
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << ", ";
  }

  // member: power
  {
    out << "power: ";
    rosidl_generator_traits::value_to_yaml(msg.power, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Setmovemin_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: gripper_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gripper_id: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper_id, out);
    out << "\n";
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }

  // member: power
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "power: ";
    rosidl_generator_traits::value_to_yaml(msg.power, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Setmovemin_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace service_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use service_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const service_interfaces::srv::Setmovemin_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  service_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use service_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const service_interfaces::srv::Setmovemin_Request & msg)
{
  return service_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<service_interfaces::srv::Setmovemin_Request>()
{
  return "service_interfaces::srv::Setmovemin_Request";
}

template<>
inline const char * name<service_interfaces::srv::Setmovemin_Request>()
{
  return "service_interfaces/srv/Setmovemin_Request";
}

template<>
struct has_fixed_size<service_interfaces::srv::Setmovemin_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<service_interfaces::srv::Setmovemin_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<service_interfaces::srv::Setmovemin_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace service_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Setmovemin_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: movemin_accepted
  {
    out << "movemin_accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.movemin_accepted, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Setmovemin_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: movemin_accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "movemin_accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.movemin_accepted, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Setmovemin_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace service_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use service_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const service_interfaces::srv::Setmovemin_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  service_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use service_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const service_interfaces::srv::Setmovemin_Response & msg)
{
  return service_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<service_interfaces::srv::Setmovemin_Response>()
{
  return "service_interfaces::srv::Setmovemin_Response";
}

template<>
inline const char * name<service_interfaces::srv::Setmovemin_Response>()
{
  return "service_interfaces/srv/Setmovemin_Response";
}

template<>
struct has_fixed_size<service_interfaces::srv::Setmovemin_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<service_interfaces::srv::Setmovemin_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<service_interfaces::srv::Setmovemin_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<service_interfaces::srv::Setmovemin>()
{
  return "service_interfaces::srv::Setmovemin";
}

template<>
inline const char * name<service_interfaces::srv::Setmovemin>()
{
  return "service_interfaces/srv/Setmovemin";
}

template<>
struct has_fixed_size<service_interfaces::srv::Setmovemin>
  : std::integral_constant<
    bool,
    has_fixed_size<service_interfaces::srv::Setmovemin_Request>::value &&
    has_fixed_size<service_interfaces::srv::Setmovemin_Response>::value
  >
{
};

template<>
struct has_bounded_size<service_interfaces::srv::Setmovemin>
  : std::integral_constant<
    bool,
    has_bounded_size<service_interfaces::srv::Setmovemin_Request>::value &&
    has_bounded_size<service_interfaces::srv::Setmovemin_Response>::value
  >
{
};

template<>
struct is_service<service_interfaces::srv::Setmovemin>
  : std::true_type
{
};

template<>
struct is_service_request<service_interfaces::srv::Setmovemin_Request>
  : std::true_type
{
};

template<>
struct is_service_response<service_interfaces::srv::Setmovemin_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SERVICE_INTERFACES__SRV__DETAIL__SETMOVEMIN__TRAITS_HPP_
