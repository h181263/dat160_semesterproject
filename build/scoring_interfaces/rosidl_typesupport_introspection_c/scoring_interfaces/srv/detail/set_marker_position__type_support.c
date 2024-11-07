// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from scoring_interfaces:srv/SetMarkerPosition.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "scoring_interfaces/srv/detail/set_marker_position__rosidl_typesupport_introspection_c.h"
#include "scoring_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "scoring_interfaces/srv/detail/set_marker_position__functions.h"
#include "scoring_interfaces/srv/detail/set_marker_position__struct.h"


// Include directives for member types
// Member `marker_position`
#include "geometry_msgs/msg/point.h"
// Member `marker_position`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void SetMarkerPosition_Request__rosidl_typesupport_introspection_c__SetMarkerPosition_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  scoring_interfaces__srv__SetMarkerPosition_Request__init(message_memory);
}

void SetMarkerPosition_Request__rosidl_typesupport_introspection_c__SetMarkerPosition_Request_fini_function(void * message_memory)
{
  scoring_interfaces__srv__SetMarkerPosition_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember SetMarkerPosition_Request__rosidl_typesupport_introspection_c__SetMarkerPosition_Request_message_member_array[2] = {
  {
    "marker_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scoring_interfaces__srv__SetMarkerPosition_Request, marker_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "marker_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scoring_interfaces__srv__SetMarkerPosition_Request, marker_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SetMarkerPosition_Request__rosidl_typesupport_introspection_c__SetMarkerPosition_Request_message_members = {
  "scoring_interfaces__srv",  // message namespace
  "SetMarkerPosition_Request",  // message name
  2,  // number of fields
  sizeof(scoring_interfaces__srv__SetMarkerPosition_Request),
  SetMarkerPosition_Request__rosidl_typesupport_introspection_c__SetMarkerPosition_Request_message_member_array,  // message members
  SetMarkerPosition_Request__rosidl_typesupport_introspection_c__SetMarkerPosition_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  SetMarkerPosition_Request__rosidl_typesupport_introspection_c__SetMarkerPosition_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SetMarkerPosition_Request__rosidl_typesupport_introspection_c__SetMarkerPosition_Request_message_type_support_handle = {
  0,
  &SetMarkerPosition_Request__rosidl_typesupport_introspection_c__SetMarkerPosition_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_scoring_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scoring_interfaces, srv, SetMarkerPosition_Request)() {
  SetMarkerPosition_Request__rosidl_typesupport_introspection_c__SetMarkerPosition_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!SetMarkerPosition_Request__rosidl_typesupport_introspection_c__SetMarkerPosition_Request_message_type_support_handle.typesupport_identifier) {
    SetMarkerPosition_Request__rosidl_typesupport_introspection_c__SetMarkerPosition_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SetMarkerPosition_Request__rosidl_typesupport_introspection_c__SetMarkerPosition_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "scoring_interfaces/srv/detail/set_marker_position__rosidl_typesupport_introspection_c.h"
// already included above
// #include "scoring_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "scoring_interfaces/srv/detail/set_marker_position__functions.h"
// already included above
// #include "scoring_interfaces/srv/detail/set_marker_position__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void SetMarkerPosition_Response__rosidl_typesupport_introspection_c__SetMarkerPosition_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  scoring_interfaces__srv__SetMarkerPosition_Response__init(message_memory);
}

void SetMarkerPosition_Response__rosidl_typesupport_introspection_c__SetMarkerPosition_Response_fini_function(void * message_memory)
{
  scoring_interfaces__srv__SetMarkerPosition_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember SetMarkerPosition_Response__rosidl_typesupport_introspection_c__SetMarkerPosition_Response_message_member_array[1] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scoring_interfaces__srv__SetMarkerPosition_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SetMarkerPosition_Response__rosidl_typesupport_introspection_c__SetMarkerPosition_Response_message_members = {
  "scoring_interfaces__srv",  // message namespace
  "SetMarkerPosition_Response",  // message name
  1,  // number of fields
  sizeof(scoring_interfaces__srv__SetMarkerPosition_Response),
  SetMarkerPosition_Response__rosidl_typesupport_introspection_c__SetMarkerPosition_Response_message_member_array,  // message members
  SetMarkerPosition_Response__rosidl_typesupport_introspection_c__SetMarkerPosition_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  SetMarkerPosition_Response__rosidl_typesupport_introspection_c__SetMarkerPosition_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SetMarkerPosition_Response__rosidl_typesupport_introspection_c__SetMarkerPosition_Response_message_type_support_handle = {
  0,
  &SetMarkerPosition_Response__rosidl_typesupport_introspection_c__SetMarkerPosition_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_scoring_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scoring_interfaces, srv, SetMarkerPosition_Response)() {
  if (!SetMarkerPosition_Response__rosidl_typesupport_introspection_c__SetMarkerPosition_Response_message_type_support_handle.typesupport_identifier) {
    SetMarkerPosition_Response__rosidl_typesupport_introspection_c__SetMarkerPosition_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SetMarkerPosition_Response__rosidl_typesupport_introspection_c__SetMarkerPosition_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "scoring_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "scoring_interfaces/srv/detail/set_marker_position__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers scoring_interfaces__srv__detail__set_marker_position__rosidl_typesupport_introspection_c__SetMarkerPosition_service_members = {
  "scoring_interfaces__srv",  // service namespace
  "SetMarkerPosition",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // scoring_interfaces__srv__detail__set_marker_position__rosidl_typesupport_introspection_c__SetMarkerPosition_Request_message_type_support_handle,
  NULL  // response message
  // scoring_interfaces__srv__detail__set_marker_position__rosidl_typesupport_introspection_c__SetMarkerPosition_Response_message_type_support_handle
};

static rosidl_service_type_support_t scoring_interfaces__srv__detail__set_marker_position__rosidl_typesupport_introspection_c__SetMarkerPosition_service_type_support_handle = {
  0,
  &scoring_interfaces__srv__detail__set_marker_position__rosidl_typesupport_introspection_c__SetMarkerPosition_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scoring_interfaces, srv, SetMarkerPosition_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scoring_interfaces, srv, SetMarkerPosition_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_scoring_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scoring_interfaces, srv, SetMarkerPosition)() {
  if (!scoring_interfaces__srv__detail__set_marker_position__rosidl_typesupport_introspection_c__SetMarkerPosition_service_type_support_handle.typesupport_identifier) {
    scoring_interfaces__srv__detail__set_marker_position__rosidl_typesupport_introspection_c__SetMarkerPosition_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)scoring_interfaces__srv__detail__set_marker_position__rosidl_typesupport_introspection_c__SetMarkerPosition_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scoring_interfaces, srv, SetMarkerPosition_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scoring_interfaces, srv, SetMarkerPosition_Response)()->data;
  }

  return &scoring_interfaces__srv__detail__set_marker_position__rosidl_typesupport_introspection_c__SetMarkerPosition_service_type_support_handle;
}
