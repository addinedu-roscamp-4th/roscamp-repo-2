// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mycobot_interfaces:msg/CookState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/msg/cook_state.h"


#ifndef MYCOBOT_INTERFACES__MSG__DETAIL__COOK_STATE__STRUCT_H_
#define MYCOBOT_INTERFACES__MSG__DETAIL__COOK_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'state'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/CookState in the package mycobot_interfaces.
typedef struct mycobot_interfaces__msg__CookState
{
  rosidl_runtime_c__String state;
  int32_t order_id;
} mycobot_interfaces__msg__CookState;

// Struct for a sequence of mycobot_interfaces__msg__CookState.
typedef struct mycobot_interfaces__msg__CookState__Sequence
{
  mycobot_interfaces__msg__CookState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mycobot_interfaces__msg__CookState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MYCOBOT_INTERFACES__MSG__DETAIL__COOK_STATE__STRUCT_H_
