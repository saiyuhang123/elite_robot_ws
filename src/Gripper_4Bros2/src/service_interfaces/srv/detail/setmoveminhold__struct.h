// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from service_interfaces:srv/Setmoveminhold.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_INTERFACES__SRV__DETAIL__SETMOVEMINHOLD__STRUCT_H_
#define SERVICE_INTERFACES__SRV__DETAIL__SETMOVEMINHOLD__STRUCT_H_

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

/// Struct defined in srv/Setmoveminhold in the package service_interfaces.
typedef struct service_interfaces__srv__Setmoveminhold_Request
{
  rosidl_runtime_c__String status;
  /// 夹爪ID
  int32_t gripper_id;
  int32_t speed;
  int32_t power;
} service_interfaces__srv__Setmoveminhold_Request;

// Struct for a sequence of service_interfaces__srv__Setmoveminhold_Request.
typedef struct service_interfaces__srv__Setmoveminhold_Request__Sequence
{
  service_interfaces__srv__Setmoveminhold_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} service_interfaces__srv__Setmoveminhold_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Setmoveminhold in the package service_interfaces.
typedef struct service_interfaces__srv__Setmoveminhold_Response
{
  bool moveminhold_accepted;
} service_interfaces__srv__Setmoveminhold_Response;

// Struct for a sequence of service_interfaces__srv__Setmoveminhold_Response.
typedef struct service_interfaces__srv__Setmoveminhold_Response__Sequence
{
  service_interfaces__srv__Setmoveminhold_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} service_interfaces__srv__Setmoveminhold_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SERVICE_INTERFACES__SRV__DETAIL__SETMOVEMINHOLD__STRUCT_H_
