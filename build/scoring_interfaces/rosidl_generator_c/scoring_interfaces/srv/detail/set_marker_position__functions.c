// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from scoring_interfaces:srv/SetMarkerPosition.idl
// generated code does not contain a copyright notice
#include "scoring_interfaces/srv/detail/set_marker_position__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `marker_position`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
scoring_interfaces__srv__SetMarkerPosition_Request__init(scoring_interfaces__srv__SetMarkerPosition_Request * msg)
{
  if (!msg) {
    return false;
  }
  // marker_id
  // marker_position
  if (!geometry_msgs__msg__Point__init(&msg->marker_position)) {
    scoring_interfaces__srv__SetMarkerPosition_Request__fini(msg);
    return false;
  }
  return true;
}

void
scoring_interfaces__srv__SetMarkerPosition_Request__fini(scoring_interfaces__srv__SetMarkerPosition_Request * msg)
{
  if (!msg) {
    return;
  }
  // marker_id
  // marker_position
  geometry_msgs__msg__Point__fini(&msg->marker_position);
}

bool
scoring_interfaces__srv__SetMarkerPosition_Request__are_equal(const scoring_interfaces__srv__SetMarkerPosition_Request * lhs, const scoring_interfaces__srv__SetMarkerPosition_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // marker_id
  if (lhs->marker_id != rhs->marker_id) {
    return false;
  }
  // marker_position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->marker_position), &(rhs->marker_position)))
  {
    return false;
  }
  return true;
}

bool
scoring_interfaces__srv__SetMarkerPosition_Request__copy(
  const scoring_interfaces__srv__SetMarkerPosition_Request * input,
  scoring_interfaces__srv__SetMarkerPosition_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // marker_id
  output->marker_id = input->marker_id;
  // marker_position
  if (!geometry_msgs__msg__Point__copy(
      &(input->marker_position), &(output->marker_position)))
  {
    return false;
  }
  return true;
}

scoring_interfaces__srv__SetMarkerPosition_Request *
scoring_interfaces__srv__SetMarkerPosition_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scoring_interfaces__srv__SetMarkerPosition_Request * msg = (scoring_interfaces__srv__SetMarkerPosition_Request *)allocator.allocate(sizeof(scoring_interfaces__srv__SetMarkerPosition_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(scoring_interfaces__srv__SetMarkerPosition_Request));
  bool success = scoring_interfaces__srv__SetMarkerPosition_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
scoring_interfaces__srv__SetMarkerPosition_Request__destroy(scoring_interfaces__srv__SetMarkerPosition_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    scoring_interfaces__srv__SetMarkerPosition_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
scoring_interfaces__srv__SetMarkerPosition_Request__Sequence__init(scoring_interfaces__srv__SetMarkerPosition_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scoring_interfaces__srv__SetMarkerPosition_Request * data = NULL;

  if (size) {
    data = (scoring_interfaces__srv__SetMarkerPosition_Request *)allocator.zero_allocate(size, sizeof(scoring_interfaces__srv__SetMarkerPosition_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = scoring_interfaces__srv__SetMarkerPosition_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        scoring_interfaces__srv__SetMarkerPosition_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
scoring_interfaces__srv__SetMarkerPosition_Request__Sequence__fini(scoring_interfaces__srv__SetMarkerPosition_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      scoring_interfaces__srv__SetMarkerPosition_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

scoring_interfaces__srv__SetMarkerPosition_Request__Sequence *
scoring_interfaces__srv__SetMarkerPosition_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scoring_interfaces__srv__SetMarkerPosition_Request__Sequence * array = (scoring_interfaces__srv__SetMarkerPosition_Request__Sequence *)allocator.allocate(sizeof(scoring_interfaces__srv__SetMarkerPosition_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = scoring_interfaces__srv__SetMarkerPosition_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
scoring_interfaces__srv__SetMarkerPosition_Request__Sequence__destroy(scoring_interfaces__srv__SetMarkerPosition_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    scoring_interfaces__srv__SetMarkerPosition_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
scoring_interfaces__srv__SetMarkerPosition_Request__Sequence__are_equal(const scoring_interfaces__srv__SetMarkerPosition_Request__Sequence * lhs, const scoring_interfaces__srv__SetMarkerPosition_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!scoring_interfaces__srv__SetMarkerPosition_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
scoring_interfaces__srv__SetMarkerPosition_Request__Sequence__copy(
  const scoring_interfaces__srv__SetMarkerPosition_Request__Sequence * input,
  scoring_interfaces__srv__SetMarkerPosition_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(scoring_interfaces__srv__SetMarkerPosition_Request);
    scoring_interfaces__srv__SetMarkerPosition_Request * data =
      (scoring_interfaces__srv__SetMarkerPosition_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!scoring_interfaces__srv__SetMarkerPosition_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          scoring_interfaces__srv__SetMarkerPosition_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!scoring_interfaces__srv__SetMarkerPosition_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
scoring_interfaces__srv__SetMarkerPosition_Response__init(scoring_interfaces__srv__SetMarkerPosition_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  return true;
}

void
scoring_interfaces__srv__SetMarkerPosition_Response__fini(scoring_interfaces__srv__SetMarkerPosition_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
}

bool
scoring_interfaces__srv__SetMarkerPosition_Response__are_equal(const scoring_interfaces__srv__SetMarkerPosition_Response * lhs, const scoring_interfaces__srv__SetMarkerPosition_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  return true;
}

bool
scoring_interfaces__srv__SetMarkerPosition_Response__copy(
  const scoring_interfaces__srv__SetMarkerPosition_Response * input,
  scoring_interfaces__srv__SetMarkerPosition_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  return true;
}

scoring_interfaces__srv__SetMarkerPosition_Response *
scoring_interfaces__srv__SetMarkerPosition_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scoring_interfaces__srv__SetMarkerPosition_Response * msg = (scoring_interfaces__srv__SetMarkerPosition_Response *)allocator.allocate(sizeof(scoring_interfaces__srv__SetMarkerPosition_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(scoring_interfaces__srv__SetMarkerPosition_Response));
  bool success = scoring_interfaces__srv__SetMarkerPosition_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
scoring_interfaces__srv__SetMarkerPosition_Response__destroy(scoring_interfaces__srv__SetMarkerPosition_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    scoring_interfaces__srv__SetMarkerPosition_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
scoring_interfaces__srv__SetMarkerPosition_Response__Sequence__init(scoring_interfaces__srv__SetMarkerPosition_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scoring_interfaces__srv__SetMarkerPosition_Response * data = NULL;

  if (size) {
    data = (scoring_interfaces__srv__SetMarkerPosition_Response *)allocator.zero_allocate(size, sizeof(scoring_interfaces__srv__SetMarkerPosition_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = scoring_interfaces__srv__SetMarkerPosition_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        scoring_interfaces__srv__SetMarkerPosition_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
scoring_interfaces__srv__SetMarkerPosition_Response__Sequence__fini(scoring_interfaces__srv__SetMarkerPosition_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      scoring_interfaces__srv__SetMarkerPosition_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

scoring_interfaces__srv__SetMarkerPosition_Response__Sequence *
scoring_interfaces__srv__SetMarkerPosition_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scoring_interfaces__srv__SetMarkerPosition_Response__Sequence * array = (scoring_interfaces__srv__SetMarkerPosition_Response__Sequence *)allocator.allocate(sizeof(scoring_interfaces__srv__SetMarkerPosition_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = scoring_interfaces__srv__SetMarkerPosition_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
scoring_interfaces__srv__SetMarkerPosition_Response__Sequence__destroy(scoring_interfaces__srv__SetMarkerPosition_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    scoring_interfaces__srv__SetMarkerPosition_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
scoring_interfaces__srv__SetMarkerPosition_Response__Sequence__are_equal(const scoring_interfaces__srv__SetMarkerPosition_Response__Sequence * lhs, const scoring_interfaces__srv__SetMarkerPosition_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!scoring_interfaces__srv__SetMarkerPosition_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
scoring_interfaces__srv__SetMarkerPosition_Response__Sequence__copy(
  const scoring_interfaces__srv__SetMarkerPosition_Response__Sequence * input,
  scoring_interfaces__srv__SetMarkerPosition_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(scoring_interfaces__srv__SetMarkerPosition_Response);
    scoring_interfaces__srv__SetMarkerPosition_Response * data =
      (scoring_interfaces__srv__SetMarkerPosition_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!scoring_interfaces__srv__SetMarkerPosition_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          scoring_interfaces__srv__SetMarkerPosition_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!scoring_interfaces__srv__SetMarkerPosition_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
