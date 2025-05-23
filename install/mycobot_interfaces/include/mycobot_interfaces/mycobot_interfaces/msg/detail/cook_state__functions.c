// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mycobot_interfaces:msg/CookState.idl
// generated code does not contain a copyright notice
#include "mycobot_interfaces/msg/detail/cook_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `state`
#include "rosidl_runtime_c/string_functions.h"

bool
mycobot_interfaces__msg__CookState__init(mycobot_interfaces__msg__CookState * msg)
{
  if (!msg) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__init(&msg->state)) {
    mycobot_interfaces__msg__CookState__fini(msg);
    return false;
  }
  // order_id
  return true;
}

void
mycobot_interfaces__msg__CookState__fini(mycobot_interfaces__msg__CookState * msg)
{
  if (!msg) {
    return;
  }
  // state
  rosidl_runtime_c__String__fini(&msg->state);
  // order_id
}

bool
mycobot_interfaces__msg__CookState__are_equal(const mycobot_interfaces__msg__CookState * lhs, const mycobot_interfaces__msg__CookState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  // order_id
  if (lhs->order_id != rhs->order_id) {
    return false;
  }
  return true;
}

bool
mycobot_interfaces__msg__CookState__copy(
  const mycobot_interfaces__msg__CookState * input,
  mycobot_interfaces__msg__CookState * output)
{
  if (!input || !output) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  // order_id
  output->order_id = input->order_id;
  return true;
}

mycobot_interfaces__msg__CookState *
mycobot_interfaces__msg__CookState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mycobot_interfaces__msg__CookState * msg = (mycobot_interfaces__msg__CookState *)allocator.allocate(sizeof(mycobot_interfaces__msg__CookState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mycobot_interfaces__msg__CookState));
  bool success = mycobot_interfaces__msg__CookState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mycobot_interfaces__msg__CookState__destroy(mycobot_interfaces__msg__CookState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mycobot_interfaces__msg__CookState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mycobot_interfaces__msg__CookState__Sequence__init(mycobot_interfaces__msg__CookState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mycobot_interfaces__msg__CookState * data = NULL;

  if (size) {
    data = (mycobot_interfaces__msg__CookState *)allocator.zero_allocate(size, sizeof(mycobot_interfaces__msg__CookState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mycobot_interfaces__msg__CookState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mycobot_interfaces__msg__CookState__fini(&data[i - 1]);
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
mycobot_interfaces__msg__CookState__Sequence__fini(mycobot_interfaces__msg__CookState__Sequence * array)
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
      mycobot_interfaces__msg__CookState__fini(&array->data[i]);
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

mycobot_interfaces__msg__CookState__Sequence *
mycobot_interfaces__msg__CookState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mycobot_interfaces__msg__CookState__Sequence * array = (mycobot_interfaces__msg__CookState__Sequence *)allocator.allocate(sizeof(mycobot_interfaces__msg__CookState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mycobot_interfaces__msg__CookState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mycobot_interfaces__msg__CookState__Sequence__destroy(mycobot_interfaces__msg__CookState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mycobot_interfaces__msg__CookState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mycobot_interfaces__msg__CookState__Sequence__are_equal(const mycobot_interfaces__msg__CookState__Sequence * lhs, const mycobot_interfaces__msg__CookState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mycobot_interfaces__msg__CookState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mycobot_interfaces__msg__CookState__Sequence__copy(
  const mycobot_interfaces__msg__CookState__Sequence * input,
  mycobot_interfaces__msg__CookState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mycobot_interfaces__msg__CookState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mycobot_interfaces__msg__CookState * data =
      (mycobot_interfaces__msg__CookState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mycobot_interfaces__msg__CookState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mycobot_interfaces__msg__CookState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mycobot_interfaces__msg__CookState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
