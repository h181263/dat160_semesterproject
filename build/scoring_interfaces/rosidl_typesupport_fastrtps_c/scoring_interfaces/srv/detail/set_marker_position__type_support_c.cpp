// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from scoring_interfaces:srv/SetMarkerPosition.idl
// generated code does not contain a copyright notice
#include "scoring_interfaces/srv/detail/set_marker_position__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "scoring_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "scoring_interfaces/srv/detail/set_marker_position__struct.h"
#include "scoring_interfaces/srv/detail/set_marker_position__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "geometry_msgs/msg/detail/point__functions.h"  // marker_position

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_scoring_interfaces
size_t get_serialized_size_geometry_msgs__msg__Point(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_scoring_interfaces
size_t max_serialized_size_geometry_msgs__msg__Point(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_scoring_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Point)();


using _SetMarkerPosition_Request__ros_msg_type = scoring_interfaces__srv__SetMarkerPosition_Request;

static bool _SetMarkerPosition_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SetMarkerPosition_Request__ros_msg_type * ros_message = static_cast<const _SetMarkerPosition_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: marker_id
  {
    cdr << ros_message->marker_id;
  }

  // Field name: marker_position
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Point
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->marker_position, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _SetMarkerPosition_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SetMarkerPosition_Request__ros_msg_type * ros_message = static_cast<_SetMarkerPosition_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: marker_id
  {
    cdr >> ros_message->marker_id;
  }

  // Field name: marker_position
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Point
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->marker_position))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scoring_interfaces
size_t get_serialized_size_scoring_interfaces__srv__SetMarkerPosition_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetMarkerPosition_Request__ros_msg_type * ros_message = static_cast<const _SetMarkerPosition_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name marker_id
  {
    size_t item_size = sizeof(ros_message->marker_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name marker_position

  current_alignment += get_serialized_size_geometry_msgs__msg__Point(
    &(ros_message->marker_position), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _SetMarkerPosition_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_scoring_interfaces__srv__SetMarkerPosition_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scoring_interfaces
size_t max_serialized_size_scoring_interfaces__srv__SetMarkerPosition_Request(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: marker_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: marker_position
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Point(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _SetMarkerPosition_Request__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_scoring_interfaces__srv__SetMarkerPosition_Request(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_SetMarkerPosition_Request = {
  "scoring_interfaces::srv",
  "SetMarkerPosition_Request",
  _SetMarkerPosition_Request__cdr_serialize,
  _SetMarkerPosition_Request__cdr_deserialize,
  _SetMarkerPosition_Request__get_serialized_size,
  _SetMarkerPosition_Request__max_serialized_size
};

static rosidl_message_type_support_t _SetMarkerPosition_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SetMarkerPosition_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scoring_interfaces, srv, SetMarkerPosition_Request)() {
  return &_SetMarkerPosition_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "scoring_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "scoring_interfaces/srv/detail/set_marker_position__struct.h"
// already included above
// #include "scoring_interfaces/srv/detail/set_marker_position__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _SetMarkerPosition_Response__ros_msg_type = scoring_interfaces__srv__SetMarkerPosition_Response;

static bool _SetMarkerPosition_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SetMarkerPosition_Response__ros_msg_type * ros_message = static_cast<const _SetMarkerPosition_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: accepted
  {
    cdr << (ros_message->accepted ? true : false);
  }

  return true;
}

static bool _SetMarkerPosition_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SetMarkerPosition_Response__ros_msg_type * ros_message = static_cast<_SetMarkerPosition_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: accepted
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->accepted = tmp ? true : false;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scoring_interfaces
size_t get_serialized_size_scoring_interfaces__srv__SetMarkerPosition_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetMarkerPosition_Response__ros_msg_type * ros_message = static_cast<const _SetMarkerPosition_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name accepted
  {
    size_t item_size = sizeof(ros_message->accepted);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SetMarkerPosition_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_scoring_interfaces__srv__SetMarkerPosition_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scoring_interfaces
size_t max_serialized_size_scoring_interfaces__srv__SetMarkerPosition_Response(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: accepted
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _SetMarkerPosition_Response__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_scoring_interfaces__srv__SetMarkerPosition_Response(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_SetMarkerPosition_Response = {
  "scoring_interfaces::srv",
  "SetMarkerPosition_Response",
  _SetMarkerPosition_Response__cdr_serialize,
  _SetMarkerPosition_Response__cdr_deserialize,
  _SetMarkerPosition_Response__get_serialized_size,
  _SetMarkerPosition_Response__max_serialized_size
};

static rosidl_message_type_support_t _SetMarkerPosition_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SetMarkerPosition_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scoring_interfaces, srv, SetMarkerPosition_Response)() {
  return &_SetMarkerPosition_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "scoring_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "scoring_interfaces/srv/set_marker_position.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t SetMarkerPosition__callbacks = {
  "scoring_interfaces::srv",
  "SetMarkerPosition",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scoring_interfaces, srv, SetMarkerPosition_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scoring_interfaces, srv, SetMarkerPosition_Response)(),
};

static rosidl_service_type_support_t SetMarkerPosition__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &SetMarkerPosition__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scoring_interfaces, srv, SetMarkerPosition)() {
  return &SetMarkerPosition__handle;
}

#if defined(__cplusplus)
}
#endif
