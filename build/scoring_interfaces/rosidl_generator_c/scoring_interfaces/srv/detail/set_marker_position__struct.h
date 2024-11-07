// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from scoring_interfaces:srv/SetMarkerPosition.idl
// generated code does not contain a copyright notice

#ifndef SCORING_INTERFACES__SRV__DETAIL__SET_MARKER_POSITION__STRUCT_H_
#define SCORING_INTERFACES__SRV__DETAIL__SET_MARKER_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'marker_position'
#include "geometry_msgs/msg/detail/point__struct.h"

// Struct defined in srv/SetMarkerPosition in the package scoring_interfaces.
typedef struct scoring_interfaces__srv__SetMarkerPosition_Request
{
  int8_t marker_id;
  geometry_msgs__msg__Point marker_position;
} scoring_interfaces__srv__SetMarkerPosition_Request;

// Struct for a sequence of scoring_interfaces__srv__SetMarkerPosition_Request.
typedef struct scoring_interfaces__srv__SetMarkerPosition_Request__Sequence
{
  scoring_interfaces__srv__SetMarkerPosition_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} scoring_interfaces__srv__SetMarkerPosition_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/SetMarkerPosition in the package scoring_interfaces.
typedef struct scoring_interfaces__srv__SetMarkerPosition_Response
{
  bool accepted;
} scoring_interfaces__srv__SetMarkerPosition_Response;

// Struct for a sequence of scoring_interfaces__srv__SetMarkerPosition_Response.
typedef struct scoring_interfaces__srv__SetMarkerPosition_Response__Sequence
{
  scoring_interfaces__srv__SetMarkerPosition_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} scoring_interfaces__srv__SetMarkerPosition_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SCORING_INTERFACES__SRV__DETAIL__SET_MARKER_POSITION__STRUCT_H_
