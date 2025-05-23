// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mycobot_interfaces:msg/CookState.idl
// generated code does not contain a copyright notice

#include "mycobot_interfaces/msg/detail/cook_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__msg__CookState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x43, 0xb7, 0xc7, 0x0c, 0xc5, 0x44, 0x3f, 0x57,
      0x0f, 0x31, 0xc3, 0xe5, 0xbd, 0xf1, 0x62, 0xbf,
      0x36, 0x0a, 0xad, 0x3d, 0xbb, 0x59, 0x7f, 0x6b,
      0x75, 0x33, 0x3d, 0xf3, 0xf3, 0xeb, 0xf1, 0x7d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char mycobot_interfaces__msg__CookState__TYPE_NAME[] = "mycobot_interfaces/msg/CookState";

// Define type names, field names, and default values
static char mycobot_interfaces__msg__CookState__FIELD_NAME__state[] = "state";
static char mycobot_interfaces__msg__CookState__FIELD_NAME__order_id[] = "order_id";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__msg__CookState__FIELDS[] = {
  {
    {mycobot_interfaces__msg__CookState__FIELD_NAME__state, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__CookState__FIELD_NAME__order_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mycobot_interfaces__msg__CookState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__msg__CookState__TYPE_NAME, 32, 32},
      {mycobot_interfaces__msg__CookState__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string state\n"
  "int32 order_id";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__msg__CookState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__msg__CookState__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 27, 27},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__msg__CookState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__msg__CookState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
