// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mycobot_interfaces:msg/MycobotSetAngles.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/msg/mycobot_set_angles.h"


#ifndef MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_SET_ANGLES__FUNCTIONS_H_
#define MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_SET_ANGLES__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "mycobot_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "mycobot_interfaces/msg/detail/mycobot_set_angles__struct.h"

/// Initialize msg/MycobotSetAngles message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mycobot_interfaces__msg__MycobotSetAngles
 * )) before or use
 * mycobot_interfaces__msg__MycobotSetAngles__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
bool
mycobot_interfaces__msg__MycobotSetAngles__init(mycobot_interfaces__msg__MycobotSetAngles * msg);

/// Finalize msg/MycobotSetAngles message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
void
mycobot_interfaces__msg__MycobotSetAngles__fini(mycobot_interfaces__msg__MycobotSetAngles * msg);

/// Create msg/MycobotSetAngles message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mycobot_interfaces__msg__MycobotSetAngles__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
mycobot_interfaces__msg__MycobotSetAngles *
mycobot_interfaces__msg__MycobotSetAngles__create(void);

/// Destroy msg/MycobotSetAngles message.
/**
 * It calls
 * mycobot_interfaces__msg__MycobotSetAngles__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
void
mycobot_interfaces__msg__MycobotSetAngles__destroy(mycobot_interfaces__msg__MycobotSetAngles * msg);

/// Check for msg/MycobotSetAngles message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
bool
mycobot_interfaces__msg__MycobotSetAngles__are_equal(const mycobot_interfaces__msg__MycobotSetAngles * lhs, const mycobot_interfaces__msg__MycobotSetAngles * rhs);

/// Copy a msg/MycobotSetAngles message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
bool
mycobot_interfaces__msg__MycobotSetAngles__copy(
  const mycobot_interfaces__msg__MycobotSetAngles * input,
  mycobot_interfaces__msg__MycobotSetAngles * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__msg__MycobotSetAngles__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
mycobot_interfaces__msg__MycobotSetAngles__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__msg__MycobotSetAngles__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__msg__MycobotSetAngles__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/MycobotSetAngles messages.
/**
 * It allocates the memory for the number of elements and calls
 * mycobot_interfaces__msg__MycobotSetAngles__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
bool
mycobot_interfaces__msg__MycobotSetAngles__Sequence__init(mycobot_interfaces__msg__MycobotSetAngles__Sequence * array, size_t size);

/// Finalize array of msg/MycobotSetAngles messages.
/**
 * It calls
 * mycobot_interfaces__msg__MycobotSetAngles__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
void
mycobot_interfaces__msg__MycobotSetAngles__Sequence__fini(mycobot_interfaces__msg__MycobotSetAngles__Sequence * array);

/// Create array of msg/MycobotSetAngles messages.
/**
 * It allocates the memory for the array and calls
 * mycobot_interfaces__msg__MycobotSetAngles__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
mycobot_interfaces__msg__MycobotSetAngles__Sequence *
mycobot_interfaces__msg__MycobotSetAngles__Sequence__create(size_t size);

/// Destroy array of msg/MycobotSetAngles messages.
/**
 * It calls
 * mycobot_interfaces__msg__MycobotSetAngles__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
void
mycobot_interfaces__msg__MycobotSetAngles__Sequence__destroy(mycobot_interfaces__msg__MycobotSetAngles__Sequence * array);

/// Check for msg/MycobotSetAngles message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
bool
mycobot_interfaces__msg__MycobotSetAngles__Sequence__are_equal(const mycobot_interfaces__msg__MycobotSetAngles__Sequence * lhs, const mycobot_interfaces__msg__MycobotSetAngles__Sequence * rhs);

/// Copy an array of msg/MycobotSetAngles messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
bool
mycobot_interfaces__msg__MycobotSetAngles__Sequence__copy(
  const mycobot_interfaces__msg__MycobotSetAngles__Sequence * input,
  mycobot_interfaces__msg__MycobotSetAngles__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_SET_ANGLES__FUNCTIONS_H_
