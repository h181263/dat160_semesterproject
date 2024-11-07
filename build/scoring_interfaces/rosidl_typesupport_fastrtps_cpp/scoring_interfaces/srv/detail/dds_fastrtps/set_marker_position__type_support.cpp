// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from scoring_interfaces:srv/SetMarkerPosition.idl
// generated code does not contain a copyright notice
#include "scoring_interfaces/srv/detail/set_marker_position__rosidl_typesupport_fastrtps_cpp.hpp"
#include "scoring_interfaces/srv/detail/set_marker_position__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Point &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Point &);
size_t get_serialized_size(
  const geometry_msgs::msg::Point &,
  size_t current_alignment);
size_t
max_serialized_size_Point(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs


namespace scoring_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_scoring_interfaces
cdr_serialize(
  const scoring_interfaces::srv::SetMarkerPosition_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: marker_id
  cdr << ros_message.marker_id;
  // Member: marker_position
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.marker_position,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_scoring_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  scoring_interfaces::srv::SetMarkerPosition_Request & ros_message)
{
  // Member: marker_id
  cdr >> ros_message.marker_id;

  // Member: marker_position
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.marker_position);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_scoring_interfaces
get_serialized_size(
  const scoring_interfaces::srv::SetMarkerPosition_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: marker_id
  {
    size_t item_size = sizeof(ros_message.marker_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: marker_position

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.marker_position, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_scoring_interfaces
max_serialized_size_SetMarkerPosition_Request(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: marker_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: marker_position
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Point(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _SetMarkerPosition_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const scoring_interfaces::srv::SetMarkerPosition_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SetMarkerPosition_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<scoring_interfaces::srv::SetMarkerPosition_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SetMarkerPosition_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const scoring_interfaces::srv::SetMarkerPosition_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SetMarkerPosition_Request__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_SetMarkerPosition_Request(full_bounded, 0);
}

static message_type_support_callbacks_t _SetMarkerPosition_Request__callbacks = {
  "scoring_interfaces::srv",
  "SetMarkerPosition_Request",
  _SetMarkerPosition_Request__cdr_serialize,
  _SetMarkerPosition_Request__cdr_deserialize,
  _SetMarkerPosition_Request__get_serialized_size,
  _SetMarkerPosition_Request__max_serialized_size
};

static rosidl_message_type_support_t _SetMarkerPosition_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SetMarkerPosition_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace scoring_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_scoring_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<scoring_interfaces::srv::SetMarkerPosition_Request>()
{
  return &scoring_interfaces::srv::typesupport_fastrtps_cpp::_SetMarkerPosition_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, scoring_interfaces, srv, SetMarkerPosition_Request)() {
  return &scoring_interfaces::srv::typesupport_fastrtps_cpp::_SetMarkerPosition_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace scoring_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_scoring_interfaces
cdr_serialize(
  const scoring_interfaces::srv::SetMarkerPosition_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: accepted
  cdr << (ros_message.accepted ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_scoring_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  scoring_interfaces::srv::SetMarkerPosition_Response & ros_message)
{
  // Member: accepted
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.accepted = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_scoring_interfaces
get_serialized_size(
  const scoring_interfaces::srv::SetMarkerPosition_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: accepted
  {
    size_t item_size = sizeof(ros_message.accepted);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_scoring_interfaces
max_serialized_size_SetMarkerPosition_Response(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: accepted
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _SetMarkerPosition_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const scoring_interfaces::srv::SetMarkerPosition_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SetMarkerPosition_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<scoring_interfaces::srv::SetMarkerPosition_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SetMarkerPosition_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const scoring_interfaces::srv::SetMarkerPosition_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SetMarkerPosition_Response__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_SetMarkerPosition_Response(full_bounded, 0);
}

static message_type_support_callbacks_t _SetMarkerPosition_Response__callbacks = {
  "scoring_interfaces::srv",
  "SetMarkerPosition_Response",
  _SetMarkerPosition_Response__cdr_serialize,
  _SetMarkerPosition_Response__cdr_deserialize,
  _SetMarkerPosition_Response__get_serialized_size,
  _SetMarkerPosition_Response__max_serialized_size
};

static rosidl_message_type_support_t _SetMarkerPosition_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SetMarkerPosition_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace scoring_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_scoring_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<scoring_interfaces::srv::SetMarkerPosition_Response>()
{
  return &scoring_interfaces::srv::typesupport_fastrtps_cpp::_SetMarkerPosition_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, scoring_interfaces, srv, SetMarkerPosition_Response)() {
  return &scoring_interfaces::srv::typesupport_fastrtps_cpp::_SetMarkerPosition_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace scoring_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _SetMarkerPosition__callbacks = {
  "scoring_interfaces::srv",
  "SetMarkerPosition",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, scoring_interfaces, srv, SetMarkerPosition_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, scoring_interfaces, srv, SetMarkerPosition_Response)(),
};

static rosidl_service_type_support_t _SetMarkerPosition__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SetMarkerPosition__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace scoring_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_scoring_interfaces
const rosidl_service_type_support_t *
get_service_type_support_handle<scoring_interfaces::srv::SetMarkerPosition>()
{
  return &scoring_interfaces::srv::typesupport_fastrtps_cpp::_SetMarkerPosition__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, scoring_interfaces, srv, SetMarkerPosition)() {
  return &scoring_interfaces::srv::typesupport_fastrtps_cpp::_SetMarkerPosition__handle;
}

#ifdef __cplusplus
}
#endif
