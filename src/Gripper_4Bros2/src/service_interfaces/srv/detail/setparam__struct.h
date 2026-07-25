// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from service_interfaces:srv/Setparam.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_INTERFACES__SRV__DETAIL__SETPARAM__STRUCT_H_
#define SERVICE_INTERFACES__SRV__DETAIL__SETPARAM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'status'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Setparam in the package service_interfaces.
typedef struct service_interfaces__srv__Setparam_Request
{
  rosidl_runtime_c__String status;
  /// 夹爪ID
  int32_t gripper_id;
} service_interfaces__srv__Setparam_Request;

// Struct for a sequence of service_interfaces__srv__Setparam_Request.
typedef struct service_interfaces__srv__Setparam_Request__Sequence
{
  service_interfaces__srv__Setparam_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} service_interfaces__srv__Setparam_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Setparam in the package service_interfaces.
typedef struct service_interfaces__srv__Setparam_Response
{
  bool param_accepted;
} service_interfaces__srv__Setparam_Response;

// Struct for a sequence of service_interfaces__srv__Setparam_Response.
typedef struct service_interfaces__srv__Setparam_Response__Sequence
{
  service_interfaces__srv__Setparam_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} service_interfaces__srv__Setparam_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SERVICE_INTERFACES__SRV__DETAIL__SETPARAM__STRUCT_H_
