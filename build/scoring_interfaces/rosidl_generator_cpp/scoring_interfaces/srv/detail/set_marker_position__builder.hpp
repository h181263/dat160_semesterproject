// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from scoring_interfaces:srv/SetMarkerPosition.idl
// generated code does not contain a copyright notice

#ifndef SCORING_INTERFACES__SRV__DETAIL__SET_MARKER_POSITION__BUILDER_HPP_
#define SCORING_INTERFACES__SRV__DETAIL__SET_MARKER_POSITION__BUILDER_HPP_

#include "scoring_interfaces/srv/detail/set_marker_position__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace scoring_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetMarkerPosition_Request_marker_position
{
public:
  explicit Init_SetMarkerPosition_Request_marker_position(::scoring_interfaces::srv::SetMarkerPosition_Request & msg)
  : msg_(msg)
  {}
  ::scoring_interfaces::srv::SetMarkerPosition_Request marker_position(::scoring_interfaces::srv::SetMarkerPosition_Request::_marker_position_type arg)
  {
    msg_.marker_position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scoring_interfaces::srv::SetMarkerPosition_Request msg_;
};

class Init_SetMarkerPosition_Request_marker_id
{
public:
  Init_SetMarkerPosition_Request_marker_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMarkerPosition_Request_marker_position marker_id(::scoring_interfaces::srv::SetMarkerPosition_Request::_marker_id_type arg)
  {
    msg_.marker_id = std::move(arg);
    return Init_SetMarkerPosition_Request_marker_position(msg_);
  }

private:
  ::scoring_interfaces::srv::SetMarkerPosition_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::scoring_interfaces::srv::SetMarkerPosition_Request>()
{
  return scoring_interfaces::srv::builder::Init_SetMarkerPosition_Request_marker_id();
}

}  // namespace scoring_interfaces


namespace scoring_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetMarkerPosition_Response_accepted
{
public:
  Init_SetMarkerPosition_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::scoring_interfaces::srv::SetMarkerPosition_Response accepted(::scoring_interfaces::srv::SetMarkerPosition_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scoring_interfaces::srv::SetMarkerPosition_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::scoring_interfaces::srv::SetMarkerPosition_Response>()
{
  return scoring_interfaces::srv::builder::Init_SetMarkerPosition_Response_accepted();
}

}  // namespace scoring_interfaces

#endif  // SCORING_INTERFACES__SRV__DETAIL__SET_MARKER_POSITION__BUILDER_HPP_
