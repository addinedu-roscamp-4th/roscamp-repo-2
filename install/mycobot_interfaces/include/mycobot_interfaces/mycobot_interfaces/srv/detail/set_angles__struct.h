// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mycobot_interfaces:srv/SetAngles.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/srv/set_angles.h"


#ifndef MYCOBOT_INTERFACES__SRV__DETAIL__SET_ANGLES__STRUCT_H_
#define MYCOBOT_INTERFACES__SRV__DETAIL__SET_ANGLES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetAngles in the package mycobot_interfaces.
typedef struct mycobot_interfaces__srv__SetAngles_Request
{
  float joint_1;
  float joint_2;
  float joint_3;
  float joint_4;
  float joint_5;
  float joint_6;
  int8_t speed;
} mycobot_interfaces__srv__SetAngles_Request;

// Struct for a sequence of mycobot_interfaces__srv__SetAngles_Request.
typedef struct mycobot_interfaces__srv__SetAngles_Request__Sequence
{
  mycobot_interfaces__srv__SetAngles_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mycobot_interfaces__srv__SetAngles_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/SetAngles in the package mycobot_interfaces.
typedef struct mycobot_interfaces__srv__SetAngles_Response
{
  bool flag;
} mycobot_interfaces__srv__SetAngles_Response;

// Struct for a sequence of mycobot_interfaces__srv__SetAngles_Response.
typedef struct mycobot_interfaces__srv__SetAngles_Response__Sequence
{
  mycobot_interfaces__srv__SetAngles_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mycobot_interfaces__srv__SetAngles_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mycobot_interfaces__srv__SetAngles_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mycobot_interfaces__srv__SetAngles_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/SetAngles in the package mycobot_interfaces.
typedef struct mycobot_interfaces__srv__SetAngles_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mycobot_interfaces__srv__SetAngles_Request__Sequence request;
  mycobot_interfaces__srv__SetAngles_Response__Sequence response;
} mycobot_interfaces__srv__SetAngles_Event;

// Struct for a sequence of mycobot_interfaces__srv__SetAngles_Event.
typedef struct mycobot_interfaces__srv__SetAngles_Event__Sequence
{
  mycobot_interfaces__srv__SetAngles_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mycobot_interfaces__srv__SetAngles_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MYCOBOT_INTERFACES__SRV__DETAIL__SET_ANGLES__STRUCT_H_
