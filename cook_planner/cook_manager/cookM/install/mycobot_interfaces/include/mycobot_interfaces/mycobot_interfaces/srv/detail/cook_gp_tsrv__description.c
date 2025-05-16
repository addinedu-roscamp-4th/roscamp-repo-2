// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mycobot_interfaces:srv/CookGPTsrv.idl
// generated code does not contain a copyright notice

#include "mycobot_interfaces/srv/detail/cook_gp_tsrv__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__srv__CookGPTsrv__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfe, 0x7b, 0x8c, 0xe9, 0xa7, 0x30, 0xe1, 0x37,
      0xbb, 0xe9, 0x5b, 0xa9, 0x3e, 0xc2, 0x3b, 0xcb,
      0xad, 0x84, 0xf3, 0xf1, 0xae, 0xea, 0xcf, 0x97,
      0xc1, 0xf1, 0xa9, 0x93, 0x01, 0x3b, 0x91, 0xc4,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__srv__CookGPTsrv_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x02, 0xef, 0x3e, 0x9c, 0x69, 0x4b, 0x20, 0xa8,
      0x27, 0xb1, 0x22, 0xb7, 0xdf, 0xf5, 0xb8, 0x77,
      0x11, 0x4b, 0x7f, 0x51, 0x7e, 0xe2, 0xce, 0x4f,
      0xa6, 0xd4, 0x41, 0x1f, 0x24, 0x50, 0x4e, 0xa4,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__srv__CookGPTsrv_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x44, 0xa0, 0x82, 0x5d, 0x30, 0x4c, 0x9e, 0x2e,
      0x1b, 0x20, 0x3d, 0xd6, 0x62, 0x50, 0xd8, 0x07,
      0xe2, 0xde, 0xa3, 0xf4, 0x60, 0x24, 0x7d, 0xd0,
      0x49, 0x1e, 0x68, 0x6a, 0xeb, 0xb8, 0xdd, 0xdb,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__srv__CookGPTsrv_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x73, 0x06, 0x5f, 0xed, 0x3c, 0x91, 0x7b, 0xf0,
      0x5d, 0x41, 0xe7, 0x1a, 0x86, 0x9b, 0xfd, 0xd6,
      0x5f, 0x0d, 0x1b, 0x4f, 0xd9, 0x1c, 0xb6, 0xbd,
      0x31, 0x70, 0xa9, 0xf4, 0xb9, 0x2f, 0xac, 0x32,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char mycobot_interfaces__srv__CookGPTsrv__TYPE_NAME[] = "mycobot_interfaces/srv/CookGPTsrv";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char mycobot_interfaces__srv__CookGPTsrv_Event__TYPE_NAME[] = "mycobot_interfaces/srv/CookGPTsrv_Event";
static char mycobot_interfaces__srv__CookGPTsrv_Request__TYPE_NAME[] = "mycobot_interfaces/srv/CookGPTsrv_Request";
static char mycobot_interfaces__srv__CookGPTsrv_Response__TYPE_NAME[] = "mycobot_interfaces/srv/CookGPTsrv_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char mycobot_interfaces__srv__CookGPTsrv__FIELD_NAME__request_message[] = "request_message";
static char mycobot_interfaces__srv__CookGPTsrv__FIELD_NAME__response_message[] = "response_message";
static char mycobot_interfaces__srv__CookGPTsrv__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__srv__CookGPTsrv__FIELDS[] = {
  {
    {mycobot_interfaces__srv__CookGPTsrv__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mycobot_interfaces__srv__CookGPTsrv_Request__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__CookGPTsrv__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mycobot_interfaces__srv__CookGPTsrv_Response__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__CookGPTsrv__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mycobot_interfaces__srv__CookGPTsrv_Event__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mycobot_interfaces__srv__CookGPTsrv__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__CookGPTsrv_Event__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__CookGPTsrv_Request__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__CookGPTsrv_Response__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mycobot_interfaces__srv__CookGPTsrv__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__srv__CookGPTsrv__TYPE_NAME, 33, 33},
      {mycobot_interfaces__srv__CookGPTsrv__FIELDS, 3, 3},
    },
    {mycobot_interfaces__srv__CookGPTsrv__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = mycobot_interfaces__srv__CookGPTsrv_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = mycobot_interfaces__srv__CookGPTsrv_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = mycobot_interfaces__srv__CookGPTsrv_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mycobot_interfaces__srv__CookGPTsrv_Request__FIELD_NAME__command[] = "command";
static char mycobot_interfaces__srv__CookGPTsrv_Request__FIELD_NAME__robot_id[] = "robot_id";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__srv__CookGPTsrv_Request__FIELDS[] = {
  {
    {mycobot_interfaces__srv__CookGPTsrv_Request__FIELD_NAME__command, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__CookGPTsrv_Request__FIELD_NAME__robot_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mycobot_interfaces__srv__CookGPTsrv_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__srv__CookGPTsrv_Request__TYPE_NAME, 41, 41},
      {mycobot_interfaces__srv__CookGPTsrv_Request__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mycobot_interfaces__srv__CookGPTsrv_Response__FIELD_NAME__x[] = "x";
static char mycobot_interfaces__srv__CookGPTsrv_Response__FIELD_NAME__y[] = "y";
static char mycobot_interfaces__srv__CookGPTsrv_Response__FIELD_NAME__z[] = "z";
static char mycobot_interfaces__srv__CookGPTsrv_Response__FIELD_NAME__rx[] = "rx";
static char mycobot_interfaces__srv__CookGPTsrv_Response__FIELD_NAME__ry[] = "ry";
static char mycobot_interfaces__srv__CookGPTsrv_Response__FIELD_NAME__rz[] = "rz";
static char mycobot_interfaces__srv__CookGPTsrv_Response__FIELD_NAME__dish[] = "dish";
static char mycobot_interfaces__srv__CookGPTsrv_Response__FIELD_NAME__sauce[] = "sauce";
static char mycobot_interfaces__srv__CookGPTsrv_Response__FIELD_NAME__stain[] = "stain";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__srv__CookGPTsrv_Response__FIELDS[] = {
  {
    {mycobot_interfaces__srv__CookGPTsrv_Response__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__CookGPTsrv_Response__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__CookGPTsrv_Response__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__CookGPTsrv_Response__FIELD_NAME__rx, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__CookGPTsrv_Response__FIELD_NAME__ry, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__CookGPTsrv_Response__FIELD_NAME__rz, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__CookGPTsrv_Response__FIELD_NAME__dish, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__CookGPTsrv_Response__FIELD_NAME__sauce, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__CookGPTsrv_Response__FIELD_NAME__stain, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mycobot_interfaces__srv__CookGPTsrv_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__srv__CookGPTsrv_Response__TYPE_NAME, 42, 42},
      {mycobot_interfaces__srv__CookGPTsrv_Response__FIELDS, 9, 9},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mycobot_interfaces__srv__CookGPTsrv_Event__FIELD_NAME__info[] = "info";
static char mycobot_interfaces__srv__CookGPTsrv_Event__FIELD_NAME__request[] = "request";
static char mycobot_interfaces__srv__CookGPTsrv_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__srv__CookGPTsrv_Event__FIELDS[] = {
  {
    {mycobot_interfaces__srv__CookGPTsrv_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__CookGPTsrv_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mycobot_interfaces__srv__CookGPTsrv_Request__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__CookGPTsrv_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mycobot_interfaces__srv__CookGPTsrv_Response__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mycobot_interfaces__srv__CookGPTsrv_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__CookGPTsrv_Request__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__CookGPTsrv_Response__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mycobot_interfaces__srv__CookGPTsrv_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__srv__CookGPTsrv_Event__TYPE_NAME, 39, 39},
      {mycobot_interfaces__srv__CookGPTsrv_Event__FIELDS, 3, 3},
    },
    {mycobot_interfaces__srv__CookGPTsrv_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = mycobot_interfaces__srv__CookGPTsrv_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = mycobot_interfaces__srv__CookGPTsrv_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int8 command\n"
  "string robot_id\n"
  "---\n"
  "float32 x\n"
  "float32 y\n"
  "float32 z\n"
  "float32 rx\n"
  "float32 ry\n"
  "float32 rz\n"
  "bool dish\n"
  "bool sauce\n"
  "bool stain";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__srv__CookGPTsrv__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__srv__CookGPTsrv__TYPE_NAME, 33, 33},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 127, 127},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__srv__CookGPTsrv_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__srv__CookGPTsrv_Request__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__srv__CookGPTsrv_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__srv__CookGPTsrv_Response__TYPE_NAME, 42, 42},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__srv__CookGPTsrv_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__srv__CookGPTsrv_Event__TYPE_NAME, 39, 39},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__srv__CookGPTsrv__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__srv__CookGPTsrv__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mycobot_interfaces__srv__CookGPTsrv_Event__get_individual_type_description_source(NULL);
    sources[3] = *mycobot_interfaces__srv__CookGPTsrv_Request__get_individual_type_description_source(NULL);
    sources[4] = *mycobot_interfaces__srv__CookGPTsrv_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__srv__CookGPTsrv_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__srv__CookGPTsrv_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__srv__CookGPTsrv_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__srv__CookGPTsrv_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__srv__CookGPTsrv_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__srv__CookGPTsrv_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mycobot_interfaces__srv__CookGPTsrv_Request__get_individual_type_description_source(NULL);
    sources[3] = *mycobot_interfaces__srv__CookGPTsrv_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
