// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from service_interfaces:srv/Getcopen.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_INTERFACES__SRV__DETAIL__GETCOPEN__TRAITS_HPP_
#define SERVICE_INTERFACES__SRV__DETAIL__GETCOPEN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "service_interfaces/srv/detail/getcopen__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace service_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Getcopen_Request & msg,
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Getcopen_Request & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Getcopen_Request & msg, bool use_flow_style = false)
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
  const service_interfaces::srv::Getcopen_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  service_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use service_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const service_interfaces::srv::Getcopen_Request & msg)
{
  return service_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<service_interfaces::srv::Getcopen_Request>()
{
  return "service_interfaces::srv::Getcopen_Request";
}

template<>
inline const char * name<service_interfaces::srv::Getcopen_Request>()
{
  return "service_interfaces/srv/Getcopen_Request";
}

template<>
struct has_fixed_size<service_interfaces::srv::Getcopen_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<service_interfaces::srv::Getcopen_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<service_interfaces::srv::Getcopen_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace service_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Getcopen_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: copen
  {
    out << "copen: ";
    rosidl_generator_traits::value_to_yaml(msg.copen, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Getcopen_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: copen
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "copen: ";
    rosidl_generator_traits::value_to_yaml(msg.copen, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Getcopen_Response & msg, bool use_flow_style = false)
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
  const service_interfaces::srv::Getcopen_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  service_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use service_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const service_interfaces::srv::Getcopen_Response & msg)
{
  return service_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<service_interfaces::srv::Getcopen_Response>()
{
  return "service_interfaces::srv::Getcopen_Response";
}

template<>
inline const char * name<service_interfaces::srv::Getcopen_Response>()
{
  return "service_interfaces/srv/Getcopen_Response";
}

template<>
struct has_fixed_size<service_interfaces::srv::Getcopen_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<service_interfaces::srv::Getcopen_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<service_interfaces::srv::Getcopen_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<service_interfaces::srv::Getcopen>()
{
  return "service_interfaces::srv::Getcopen";
}

template<>
inline const char * name<service_interfaces::srv::Getcopen>()
{
  return "service_interfaces/srv/Getcopen";
}

template<>
struct has_fixed_size<service_interfaces::srv::Getcopen>
  : std::integral_constant<
    bool,
    has_fixed_size<service_interfaces::srv::Getcopen_Request>::value &&
    has_fixed_size<service_interfaces::srv::Getcopen_Response>::value
  >
{
};

template<>
struct has_bounded_size<service_interfaces::srv::Getcopen>
  : std::integral_constant<
    bool,
    has_bounded_size<service_interfaces::srv::Getcopen_Request>::value &&
    has_bounded_size<service_interfaces::srv::Getcopen_Response>::value
  >
{
};

template<>
struct is_service<service_interfaces::srv::Getcopen>
  : std::true_type
{
};

template<>
struct is_service_request<service_interfaces::srv::Getcopen_Request>
  : std::true_type
{
};

template<>
struct is_service_response<service_interfaces::srv::Getcopen_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SERVICE_INTERFACES__SRV__DETAIL__GETCOPEN__TRAITS_HPP_
