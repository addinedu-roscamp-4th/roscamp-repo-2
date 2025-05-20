// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mycobot_interfaces:srv/SetCoords.idl
// generated code does not contain a copyright notice
#include "mycobot_interfaces/srv/detail/set_coords__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
mycobot_interfaces__srv__SetCoords_Request__init(mycobot_interfaces__srv__SetCoords_Request * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // z
  // rx
  // ry
  // rz
  // speed
  // model
  return true;
}

void
mycobot_interfaces__srv__SetCoords_Request__fini(mycobot_interfaces__srv__SetCoords_Request * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // z
  // rx
  // ry
  // rz
  // speed
  // model
}

bool
mycobot_interfaces__srv__SetCoords_Request__are_equal(const mycobot_interfaces__srv__SetCoords_Request * lhs, const mycobot_interfaces__srv__SetCoords_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // rx
  if (lhs->rx != rhs->rx) {
    return false;
  }
  // ry
  if (lhs->ry != rhs->ry) {
    return false;
  }
  // rz
  if (lhs->rz != rhs->rz) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  // model
  if (lhs->model != rhs->model) {
    return false;
  }
  return true;
}

bool
mycobot_interfaces__srv__SetCoords_Request__copy(
  const mycobot_interfaces__srv__SetCoords_Request * input,
  mycobot_interfaces__srv__SetCoords_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // rx
  output->rx = input->rx;
  // ry
  output->ry = input->ry;
  // rz
  output->rz = input->rz;
  // speed
  output->speed = input->speed;
  // model
  output->model = input->model;
  return true;
}

mycobot_interfaces__srv__SetCoords_Request *
mycobot_interfaces__srv__SetCoords_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mycobot_interfaces__srv__SetCoords_Request * msg = (mycobot_interfaces__srv__SetCoords_Request *)allocator.allocate(sizeof(mycobot_interfaces__srv__SetCoords_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mycobot_interfaces__srv__SetCoords_Request));
  bool success = mycobot_interfaces__srv__SetCoords_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mycobot_interfaces__srv__SetCoords_Request__destroy(mycobot_interfaces__srv__SetCoords_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mycobot_interfaces__srv__SetCoords_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mycobot_interfaces__srv__SetCoords_Request__Sequence__init(mycobot_interfaces__srv__SetCoords_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mycobot_interfaces__srv__SetCoords_Request * data = NULL;

  if (size) {
    data = (mycobot_interfaces__srv__SetCoords_Request *)allocator.zero_allocate(size, sizeof(mycobot_interfaces__srv__SetCoords_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mycobot_interfaces__srv__SetCoords_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mycobot_interfaces__srv__SetCoords_Request__fini(&data[i - 1]);
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
mycobot_interfaces__srv__SetCoords_Request__Sequence__fini(mycobot_interfaces__srv__SetCoords_Request__Sequence * array)
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
      mycobot_interfaces__srv__SetCoords_Request__fini(&array->data[i]);
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

mycobot_interfaces__srv__SetCoords_Request__Sequence *
mycobot_interfaces__srv__SetCoords_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mycobot_interfaces__srv__SetCoords_Request__Sequence * array = (mycobot_interfaces__srv__SetCoords_Request__Sequence *)allocator.allocate(sizeof(mycobot_interfaces__srv__SetCoords_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mycobot_interfaces__srv__SetCoords_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mycobot_interfaces__srv__SetCoords_Request__Sequence__destroy(mycobot_interfaces__srv__SetCoords_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mycobot_interfaces__srv__SetCoords_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mycobot_interfaces__srv__SetCoords_Request__Sequence__are_equal(const mycobot_interfaces__srv__SetCoords_Request__Sequence * lhs, const mycobot_interfaces__srv__SetCoords_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mycobot_interfaces__srv__SetCoords_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mycobot_interfaces__srv__SetCoords_Request__Sequence__copy(
  const mycobot_interfaces__srv__SetCoords_Request__Sequence * input,
  mycobot_interfaces__srv__SetCoords_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mycobot_interfaces__srv__SetCoords_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mycobot_interfaces__srv__SetCoords_Request * data =
      (mycobot_interfaces__srv__SetCoords_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mycobot_interfaces__srv__SetCoords_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mycobot_interfaces__srv__SetCoords_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mycobot_interfaces__srv__SetCoords_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
mycobot_interfaces__srv__SetCoords_Response__init(mycobot_interfaces__srv__SetCoords_Response * msg)
{
  if (!msg) {
    return false;
  }
  // flag
  return true;
}

void
mycobot_interfaces__srv__SetCoords_Response__fini(mycobot_interfaces__srv__SetCoords_Response * msg)
{
  if (!msg) {
    return;
  }
  // flag
}

bool
mycobot_interfaces__srv__SetCoords_Response__are_equal(const mycobot_interfaces__srv__SetCoords_Response * lhs, const mycobot_interfaces__srv__SetCoords_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // flag
  if (lhs->flag != rhs->flag) {
    return false;
  }
  return true;
}

bool
mycobot_interfaces__srv__SetCoords_Response__copy(
  const mycobot_interfaces__srv__SetCoords_Response * input,
  mycobot_interfaces__srv__SetCoords_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // flag
  output->flag = input->flag;
  return true;
}

mycobot_interfaces__srv__SetCoords_Response *
mycobot_interfaces__srv__SetCoords_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mycobot_interfaces__srv__SetCoords_Response * msg = (mycobot_interfaces__srv__SetCoords_Response *)allocator.allocate(sizeof(mycobot_interfaces__srv__SetCoords_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mycobot_interfaces__srv__SetCoords_Response));
  bool success = mycobot_interfaces__srv__SetCoords_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mycobot_interfaces__srv__SetCoords_Response__destroy(mycobot_interfaces__srv__SetCoords_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mycobot_interfaces__srv__SetCoords_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mycobot_interfaces__srv__SetCoords_Response__Sequence__init(mycobot_interfaces__srv__SetCoords_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mycobot_interfaces__srv__SetCoords_Response * data = NULL;

  if (size) {
    data = (mycobot_interfaces__srv__SetCoords_Response *)allocator.zero_allocate(size, sizeof(mycobot_interfaces__srv__SetCoords_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mycobot_interfaces__srv__SetCoords_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mycobot_interfaces__srv__SetCoords_Response__fini(&data[i - 1]);
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
mycobot_interfaces__srv__SetCoords_Response__Sequence__fini(mycobot_interfaces__srv__SetCoords_Response__Sequence * array)
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
      mycobot_interfaces__srv__SetCoords_Response__fini(&array->data[i]);
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

mycobot_interfaces__srv__SetCoords_Response__Sequence *
mycobot_interfaces__srv__SetCoords_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mycobot_interfaces__srv__SetCoords_Response__Sequence * array = (mycobot_interfaces__srv__SetCoords_Response__Sequence *)allocator.allocate(sizeof(mycobot_interfaces__srv__SetCoords_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mycobot_interfaces__srv__SetCoords_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mycobot_interfaces__srv__SetCoords_Response__Sequence__destroy(mycobot_interfaces__srv__SetCoords_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mycobot_interfaces__srv__SetCoords_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mycobot_interfaces__srv__SetCoords_Response__Sequence__are_equal(const mycobot_interfaces__srv__SetCoords_Response__Sequence * lhs, const mycobot_interfaces__srv__SetCoords_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mycobot_interfaces__srv__SetCoords_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mycobot_interfaces__srv__SetCoords_Response__Sequence__copy(
  const mycobot_interfaces__srv__SetCoords_Response__Sequence * input,
  mycobot_interfaces__srv__SetCoords_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mycobot_interfaces__srv__SetCoords_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mycobot_interfaces__srv__SetCoords_Response * data =
      (mycobot_interfaces__srv__SetCoords_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mycobot_interfaces__srv__SetCoords_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mycobot_interfaces__srv__SetCoords_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mycobot_interfaces__srv__SetCoords_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "mycobot_interfaces/srv/detail/set_coords__functions.h"

bool
mycobot_interfaces__srv__SetCoords_Event__init(mycobot_interfaces__srv__SetCoords_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    mycobot_interfaces__srv__SetCoords_Event__fini(msg);
    return false;
  }
  // request
  if (!mycobot_interfaces__srv__SetCoords_Request__Sequence__init(&msg->request, 0)) {
    mycobot_interfaces__srv__SetCoords_Event__fini(msg);
    return false;
  }
  // response
  if (!mycobot_interfaces__srv__SetCoords_Response__Sequence__init(&msg->response, 0)) {
    mycobot_interfaces__srv__SetCoords_Event__fini(msg);
    return false;
  }
  return true;
}

void
mycobot_interfaces__srv__SetCoords_Event__fini(mycobot_interfaces__srv__SetCoords_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  mycobot_interfaces__srv__SetCoords_Request__Sequence__fini(&msg->request);
  // response
  mycobot_interfaces__srv__SetCoords_Response__Sequence__fini(&msg->response);
}

bool
mycobot_interfaces__srv__SetCoords_Event__are_equal(const mycobot_interfaces__srv__SetCoords_Event * lhs, const mycobot_interfaces__srv__SetCoords_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!mycobot_interfaces__srv__SetCoords_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!mycobot_interfaces__srv__SetCoords_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
mycobot_interfaces__srv__SetCoords_Event__copy(
  const mycobot_interfaces__srv__SetCoords_Event * input,
  mycobot_interfaces__srv__SetCoords_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!mycobot_interfaces__srv__SetCoords_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!mycobot_interfaces__srv__SetCoords_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

mycobot_interfaces__srv__SetCoords_Event *
mycobot_interfaces__srv__SetCoords_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mycobot_interfaces__srv__SetCoords_Event * msg = (mycobot_interfaces__srv__SetCoords_Event *)allocator.allocate(sizeof(mycobot_interfaces__srv__SetCoords_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mycobot_interfaces__srv__SetCoords_Event));
  bool success = mycobot_interfaces__srv__SetCoords_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mycobot_interfaces__srv__SetCoords_Event__destroy(mycobot_interfaces__srv__SetCoords_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mycobot_interfaces__srv__SetCoords_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mycobot_interfaces__srv__SetCoords_Event__Sequence__init(mycobot_interfaces__srv__SetCoords_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mycobot_interfaces__srv__SetCoords_Event * data = NULL;

  if (size) {
    data = (mycobot_interfaces__srv__SetCoords_Event *)allocator.zero_allocate(size, sizeof(mycobot_interfaces__srv__SetCoords_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mycobot_interfaces__srv__SetCoords_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mycobot_interfaces__srv__SetCoords_Event__fini(&data[i - 1]);
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
mycobot_interfaces__srv__SetCoords_Event__Sequence__fini(mycobot_interfaces__srv__SetCoords_Event__Sequence * array)
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
      mycobot_interfaces__srv__SetCoords_Event__fini(&array->data[i]);
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

mycobot_interfaces__srv__SetCoords_Event__Sequence *
mycobot_interfaces__srv__SetCoords_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mycobot_interfaces__srv__SetCoords_Event__Sequence * array = (mycobot_interfaces__srv__SetCoords_Event__Sequence *)allocator.allocate(sizeof(mycobot_interfaces__srv__SetCoords_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mycobot_interfaces__srv__SetCoords_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mycobot_interfaces__srv__SetCoords_Event__Sequence__destroy(mycobot_interfaces__srv__SetCoords_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mycobot_interfaces__srv__SetCoords_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mycobot_interfaces__srv__SetCoords_Event__Sequence__are_equal(const mycobot_interfaces__srv__SetCoords_Event__Sequence * lhs, const mycobot_interfaces__srv__SetCoords_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mycobot_interfaces__srv__SetCoords_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mycobot_interfaces__srv__SetCoords_Event__Sequence__copy(
  const mycobot_interfaces__srv__SetCoords_Event__Sequence * input,
  mycobot_interfaces__srv__SetCoords_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mycobot_interfaces__srv__SetCoords_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mycobot_interfaces__srv__SetCoords_Event * data =
      (mycobot_interfaces__srv__SetCoords_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mycobot_interfaces__srv__SetCoords_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mycobot_interfaces__srv__SetCoords_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mycobot_interfaces__srv__SetCoords_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
