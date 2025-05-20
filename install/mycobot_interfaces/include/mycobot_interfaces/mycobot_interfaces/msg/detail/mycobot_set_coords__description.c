// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mycobot_interfaces:msg/MycobotSetCoords.idl
// generated code does not contain a copyright notice

#include "mycobot_interfaces/msg/detail/mycobot_set_coords__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__msg__MycobotSetCoords__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa9, 0x08, 0xd0, 0x10, 0x61, 0x92, 0xaa, 0xad,
      0x9b, 0x8c, 0xac, 0xb1, 0x1f, 0xa7, 0x58, 0xa7,
      0x97, 0xfc, 0xcc, 0xfa, 0xcb, 0x24, 0x8d, 0xbf,
      0x36, 0x4e, 0xe4, 0xc5, 0x78, 0xfe, 0xeb, 0xba,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char mycobot_interfaces__msg__MycobotSetCoords__TYPE_NAME[] = "mycobot_interfaces/msg/MycobotSetCoords";

// Define type names, field names, and default values
static char mycobot_interfaces__msg__MycobotSetCoords__FIELD_NAME__x[] = "x";
static char mycobot_interfaces__msg__MycobotSetCoords__FIELD_NAME__y[] = "y";
static char mycobot_interfaces__msg__MycobotSetCoords__FIELD_NAME__z[] = "z";
static char mycobot_interfaces__msg__MycobotSetCoords__FIELD_NAME__rx[] = "rx";
static char mycobot_interfaces__msg__MycobotSetCoords__FIELD_NAME__ry[] = "ry";
static char mycobot_interfaces__msg__MycobotSetCoords__FIELD_NAME__rz[] = "rz";
static char mycobot_interfaces__msg__MycobotSetCoords__FIELD_NAME__speed[] = "speed";
static char mycobot_interfaces__msg__MycobotSetCoords__FIELD_NAME__model[] = "model";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__msg__MycobotSetCoords__FIELDS[] = {
  {
    {mycobot_interfaces__msg__MycobotSetCoords__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotSetCoords__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotSetCoords__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotSetCoords__FIELD_NAME__rx, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotSetCoords__FIELD_NAME__ry, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotSetCoords__FIELD_NAME__rz, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotSetCoords__FIELD_NAME__speed, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotSetCoords__FIELD_NAME__model, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mycobot_interfaces__msg__MycobotSetCoords__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__msg__MycobotSetCoords__TYPE_NAME, 39, 39},
      {mycobot_interfaces__msg__MycobotSetCoords__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 x\n"
  "float32 y\n"
  "float32 z\n"
  "float32 rx\n"
  "float32 ry\n"
  "float32 rz\n"
  "\n"
  "int8 speed\n"
  "int8 model";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__msg__MycobotSetCoords__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__msg__MycobotSetCoords__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 86, 86},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__msg__MycobotSetCoords__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__msg__MycobotSetCoords__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
