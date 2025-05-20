// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mycobot_interfaces:msg/MycobotAngles.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/msg/mycobot_angles.h"


#ifndef MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_ANGLES__STRUCT_H_
#define MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_ANGLES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/MycobotAngles in the package mycobot_interfaces.
typedef struct mycobot_interfaces__msg__MycobotAngles
{
  float joint_1;
  float joint_2;
  float joint_3;
  float joint_4;
  float joint_5;
  float joint_6;
} mycobot_interfaces__msg__MycobotAngles;

// Struct for a sequence of mycobot_interfaces__msg__MycobotAngles.
typedef struct mycobot_interfaces__msg__MycobotAngles__Sequence
{
  mycobot_interfaces__msg__MycobotAngles * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mycobot_interfaces__msg__MycobotAngles__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_ANGLES__STRUCT_H_
