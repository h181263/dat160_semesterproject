// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from scoring_interfaces:srv/SetMarkerPosition.idl
// generated code does not contain a copyright notice

#ifndef SCORING_INTERFACES__SRV__DETAIL__SET_MARKER_POSITION__STRUCT_HPP_
#define SCORING_INTERFACES__SRV__DETAIL__SET_MARKER_POSITION__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'marker_position'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__scoring_interfaces__srv__SetMarkerPosition_Request __attribute__((deprecated))
#else
# define DEPRECATED__scoring_interfaces__srv__SetMarkerPosition_Request __declspec(deprecated)
#endif

namespace scoring_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetMarkerPosition_Request_
{
  using Type = SetMarkerPosition_Request_<ContainerAllocator>;

  explicit SetMarkerPosition_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : marker_position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->marker_id = 0;
    }
  }

  explicit SetMarkerPosition_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : marker_position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->marker_id = 0;
    }
  }

  // field types and members
  using _marker_id_type =
    int8_t;
  _marker_id_type marker_id;
  using _marker_position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _marker_position_type marker_position;

  // setters for named parameter idiom
  Type & set__marker_id(
    const int8_t & _arg)
  {
    this->marker_id = _arg;
    return *this;
  }
  Type & set__marker_position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->marker_position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    scoring_interfaces::srv::SetMarkerPosition_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const scoring_interfaces::srv::SetMarkerPosition_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<scoring_interfaces::srv::SetMarkerPosition_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<scoring_interfaces::srv::SetMarkerPosition_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      scoring_interfaces::srv::SetMarkerPosition_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<scoring_interfaces::srv::SetMarkerPosition_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      scoring_interfaces::srv::SetMarkerPosition_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<scoring_interfaces::srv::SetMarkerPosition_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<scoring_interfaces::srv::SetMarkerPosition_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<scoring_interfaces::srv::SetMarkerPosition_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__scoring_interfaces__srv__SetMarkerPosition_Request
    std::shared_ptr<scoring_interfaces::srv::SetMarkerPosition_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__scoring_interfaces__srv__SetMarkerPosition_Request
    std::shared_ptr<scoring_interfaces::srv::SetMarkerPosition_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMarkerPosition_Request_ & other) const
  {
    if (this->marker_id != other.marker_id) {
      return false;
    }
    if (this->marker_position != other.marker_position) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMarkerPosition_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMarkerPosition_Request_

// alias to use template instance with default allocator
using SetMarkerPosition_Request =
  scoring_interfaces::srv::SetMarkerPosition_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace scoring_interfaces


#ifndef _WIN32
# define DEPRECATED__scoring_interfaces__srv__SetMarkerPosition_Response __attribute__((deprecated))
#else
# define DEPRECATED__scoring_interfaces__srv__SetMarkerPosition_Response __declspec(deprecated)
#endif

namespace scoring_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetMarkerPosition_Response_
{
  using Type = SetMarkerPosition_Response_<ContainerAllocator>;

  explicit SetMarkerPosition_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit SetMarkerPosition_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    scoring_interfaces::srv::SetMarkerPosition_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const scoring_interfaces::srv::SetMarkerPosition_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<scoring_interfaces::srv::SetMarkerPosition_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<scoring_interfaces::srv::SetMarkerPosition_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      scoring_interfaces::srv::SetMarkerPosition_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<scoring_interfaces::srv::SetMarkerPosition_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      scoring_interfaces::srv::SetMarkerPosition_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<scoring_interfaces::srv::SetMarkerPosition_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<scoring_interfaces::srv::SetMarkerPosition_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<scoring_interfaces::srv::SetMarkerPosition_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__scoring_interfaces__srv__SetMarkerPosition_Response
    std::shared_ptr<scoring_interfaces::srv::SetMarkerPosition_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__scoring_interfaces__srv__SetMarkerPosition_Response
    std::shared_ptr<scoring_interfaces::srv::SetMarkerPosition_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMarkerPosition_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMarkerPosition_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMarkerPosition_Response_

// alias to use template instance with default allocator
using SetMarkerPosition_Response =
  scoring_interfaces::srv::SetMarkerPosition_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace scoring_interfaces

namespace scoring_interfaces
{

namespace srv
{

struct SetMarkerPosition
{
  using Request = scoring_interfaces::srv::SetMarkerPosition_Request;
  using Response = scoring_interfaces::srv::SetMarkerPosition_Response;
};

}  // namespace srv

}  // namespace scoring_interfaces

#endif  // SCORING_INTERFACES__SRV__DETAIL__SET_MARKER_POSITION__STRUCT_HPP_
