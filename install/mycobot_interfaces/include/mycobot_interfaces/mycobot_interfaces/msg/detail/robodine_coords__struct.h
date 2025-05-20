// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mycobot_interfaces:msg/RobodineCoords.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/msg/robodine_coords.h"


#ifndef MYCOBOT_INTERFACES__MSG__DETAIL__ROBODINE_COORDS__STRUCT_H_
#define MYCOBOT_INTERFACES__MSG__DETAIL__ROBODINE_COORDS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/RobodineCoords in the package mycobot_interfaces.
typedef struct mycobot_interfaces__msg__RobodineCoords
{
  float x;
  float y;
  float z;
  float rx;
  float ry;
  float rz;
  int32_t gripper;
  int32_t vel;
} mycobot_interfaces__msg__RobodineCoords;

// Struct for a sequence of mycobot_interfaces__msg__RobodineCoords.
typedef struct mycobot_interfaces__msg__RobodineCoords__Sequence
{
  mycobot_interfaces__msg__RobodineCoords * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mycobot_interfaces__msg__RobodineCoords__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MYCOBOT_INTERFACES__MSG__DETAIL__ROBODINE_COORDS__STRUCT_H_
