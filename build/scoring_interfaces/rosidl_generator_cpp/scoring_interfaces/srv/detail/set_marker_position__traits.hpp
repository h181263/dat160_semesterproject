// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from scoring_interfaces:srv/SetMarkerPosition.idl
// generated code does not contain a copyright notice

#ifndef SCORING_INTERFACES__SRV__DETAIL__SET_MARKER_POSITION__TRAITS_HPP_
#define SCORING_INTERFACES__SRV__DETAIL__SET_MARKER_POSITION__TRAITS_HPP_

#include "scoring_interfaces/srv/detail/set_marker_position__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'marker_position'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<scoring_interfaces::srv::SetMarkerPosition_Request>()
{
  return "scoring_interfaces::srv::SetMarkerPosition_Request";
}

template<>
inline const char * name<scoring_interfaces::srv::SetMarkerPosition_Request>()
{
  return "scoring_interfaces/srv/SetMarkerPosition_Request";
}

template<>
struct has_fixed_size<scoring_interfaces::srv::SetMarkerPosition_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value> {};

template<>
struct has_bounded_size<scoring_interfaces::srv::SetMarkerPosition_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value> {};

template<>
struct is_message<scoring_interfaces::srv::SetMarkerPosition_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<scoring_interfaces::srv::SetMarkerPosition_Response>()
{
  return "scoring_interfaces::srv::SetMarkerPosition_Response";
}

template<>
inline const char * name<scoring_interfaces::srv::SetMarkerPosition_Response>()
{
  return "scoring_interfaces/srv/SetMarkerPosition_Response";
}

template<>
struct has_fixed_size<scoring_interfaces::srv::SetMarkerPosition_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<scoring_interfaces::srv::SetMarkerPosition_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<scoring_interfaces::srv::SetMarkerPosition_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<scoring_interfaces::srv::SetMarkerPosition>()
{
  return "scoring_interfaces::srv::SetMarkerPosition";
}

template<>
inline const char * name<scoring_interfaces::srv::SetMarkerPosition>()
{
  return "scoring_interfaces/srv/SetMarkerPosition";
}

template<>
struct has_fixed_size<scoring_interfaces::srv::SetMarkerPosition>
  : std::integral_constant<
    bool,
    has_fixed_size<scoring_interfaces::srv::SetMarkerPosition_Request>::value &&
    has_fixed_size<scoring_interfaces::srv::SetMarkerPosition_Response>::value
  >
{
};

template<>
struct has_bounded_size<scoring_interfaces::srv::SetMarkerPosition>
  : std::integral_constant<
    bool,
    has_bounded_size<scoring_interfaces::srv::SetMarkerPosition_Request>::value &&
    has_bounded_size<scoring_interfaces::srv::SetMarkerPosition_Response>::value
  >
{
};

template<>
struct is_service<scoring_interfaces::srv::SetMarkerPosition>
  : std::true_type
{
};

template<>
struct is_service_request<scoring_interfaces::srv::SetMarkerPosition_Request>
  : std::true_type
{
};

template<>
struct is_service_response<scoring_interfaces::srv::SetMarkerPosition_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SCORING_INTERFACES__SRV__DETAIL__SET_MARKER_POSITION__TRAITS_HPP_
