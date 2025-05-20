// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robodine_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice
#include "robodine_msgs/msg/detail/robot_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `type`
// Member `status`
#include "rosidl_runtime_c/string_functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
robodine_msgs__msg__RobotStatus__init(robodine_msgs__msg__RobotStatus * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // type
  if (!rosidl_runtime_c__String__init(&msg->type)) {
    robodine_msgs__msg__RobotStatus__fini(msg);
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__init(&msg->status)) {
    robodine_msgs__msg__RobotStatus__fini(msg);
    return false;
  }
  // position
  if (!geometry_msgs__msg__Pose__init(&msg->position)) {
    robodine_msgs__msg__RobotStatus__fini(msg);
    return false;
  }
  // battery_level
  // timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp)) {
    robodine_msgs__msg__RobotStatus__fini(msg);
    return false;
  }
  return true;
}

void
robodine_msgs__msg__RobotStatus__fini(robodine_msgs__msg__RobotStatus * msg)
{
  if (!msg) {
    return;
  }
  // id
  // type
  rosidl_runtime_c__String__fini(&msg->type);
  // status
  rosidl_runtime_c__String__fini(&msg->status);
  // position
  geometry_msgs__msg__Pose__fini(&msg->position);
  // battery_level
  // timestamp
  builtin_interfaces__msg__Time__fini(&msg->timestamp);
}

bool
robodine_msgs__msg__RobotStatus__are_equal(const robodine_msgs__msg__RobotStatus * lhs, const robodine_msgs__msg__RobotStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->type), &(rhs->type)))
  {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status), &(rhs->status)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // battery_level
  if (lhs->battery_level != rhs->battery_level) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->timestamp), &(rhs->timestamp)))
  {
    return false;
  }
  return true;
}

bool
robodine_msgs__msg__RobotStatus__copy(
  const robodine_msgs__msg__RobotStatus * input,
  robodine_msgs__msg__RobotStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // type
  if (!rosidl_runtime_c__String__copy(
      &(input->type), &(output->type)))
  {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__copy(
      &(input->status), &(output->status)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Pose__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // battery_level
  output->battery_level = input->battery_level;
  // timestamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->timestamp), &(output->timestamp)))
  {
    return false;
  }
  return true;
}

robodine_msgs__msg__RobotStatus *
robodine_msgs__msg__RobotStatus__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robodine_msgs__msg__RobotStatus * msg = (robodine_msgs__msg__RobotStatus *)allocator.allocate(sizeof(robodine_msgs__msg__RobotStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robodine_msgs__msg__RobotStatus));
  bool success = robodine_msgs__msg__RobotStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robodine_msgs__msg__RobotStatus__destroy(robodine_msgs__msg__RobotStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robodine_msgs__msg__RobotStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robodine_msgs__msg__RobotStatus__Sequence__init(robodine_msgs__msg__RobotStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robodine_msgs__msg__RobotStatus * data = NULL;

  if (size) {
    data = (robodine_msgs__msg__RobotStatus *)allocator.zero_allocate(size, sizeof(robodine_msgs__msg__RobotStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robodine_msgs__msg__RobotStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robodine_msgs__msg__RobotStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robodine_msgs__msg__RobotStatus__Sequence__fini(robodine_msgs__msg__RobotStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robodine_msgs__msg__RobotStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robodine_msgs__msg__RobotStatus__Sequence *
robodine_msgs__msg__RobotStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robodine_msgs__msg__RobotStatus__Sequence * array = (robodine_msgs__msg__RobotStatus__Sequence *)allocator.allocate(sizeof(robodine_msgs__msg__RobotStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robodine_msgs__msg__RobotStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robodine_msgs__msg__RobotStatus__Sequence__destroy(robodine_msgs__msg__RobotStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robodine_msgs__msg__RobotStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robodine_msgs__msg__RobotStatus__Sequence__are_equal(const robodine_msgs__msg__RobotStatus__Sequence * lhs, const robodine_msgs__msg__RobotStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robodine_msgs__msg__RobotStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robodine_msgs__msg__RobotStatus__Sequence__copy(
  const robodine_msgs__msg__RobotStatus__Sequence * input,
  robodine_msgs__msg__RobotStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robodine_msgs__msg__RobotStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robodine_msgs__msg__RobotStatus * data =
      (robodine_msgs__msg__RobotStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robodine_msgs__msg__RobotStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robodine_msgs__msg__RobotStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robodine_msgs__msg__RobotStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
