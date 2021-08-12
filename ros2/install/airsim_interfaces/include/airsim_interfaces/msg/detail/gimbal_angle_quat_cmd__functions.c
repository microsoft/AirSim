// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from airsim_interfaces:msg/GimbalAngleQuatCmd.idl
// generated code does not contain a copyright notice
#include "airsim_interfaces/msg/detail/gimbal_angle_quat_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `camera_name`
// Member `vehicle_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `orientation`
#include "geometry_msgs/msg/detail/quaternion__functions.h"

bool
airsim_interfaces__msg__GimbalAngleQuatCmd__init(airsim_interfaces__msg__GimbalAngleQuatCmd * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    airsim_interfaces__msg__GimbalAngleQuatCmd__fini(msg);
    return false;
  }
  // camera_name
  if (!rosidl_runtime_c__String__init(&msg->camera_name)) {
    airsim_interfaces__msg__GimbalAngleQuatCmd__fini(msg);
    return false;
  }
  // vehicle_name
  if (!rosidl_runtime_c__String__init(&msg->vehicle_name)) {
    airsim_interfaces__msg__GimbalAngleQuatCmd__fini(msg);
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__init(&msg->orientation)) {
    airsim_interfaces__msg__GimbalAngleQuatCmd__fini(msg);
    return false;
  }
  return true;
}

void
airsim_interfaces__msg__GimbalAngleQuatCmd__fini(airsim_interfaces__msg__GimbalAngleQuatCmd * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // camera_name
  rosidl_runtime_c__String__fini(&msg->camera_name);
  // vehicle_name
  rosidl_runtime_c__String__fini(&msg->vehicle_name);
  // orientation
  geometry_msgs__msg__Quaternion__fini(&msg->orientation);
}

airsim_interfaces__msg__GimbalAngleQuatCmd *
airsim_interfaces__msg__GimbalAngleQuatCmd__create()
{
  airsim_interfaces__msg__GimbalAngleQuatCmd * msg = (airsim_interfaces__msg__GimbalAngleQuatCmd *)malloc(sizeof(airsim_interfaces__msg__GimbalAngleQuatCmd));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(airsim_interfaces__msg__GimbalAngleQuatCmd));
  bool success = airsim_interfaces__msg__GimbalAngleQuatCmd__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
airsim_interfaces__msg__GimbalAngleQuatCmd__destroy(airsim_interfaces__msg__GimbalAngleQuatCmd * msg)
{
  if (msg) {
    airsim_interfaces__msg__GimbalAngleQuatCmd__fini(msg);
  }
  free(msg);
}


bool
airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence__init(airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  airsim_interfaces__msg__GimbalAngleQuatCmd * data = NULL;
  if (size) {
    data = (airsim_interfaces__msg__GimbalAngleQuatCmd *)calloc(size, sizeof(airsim_interfaces__msg__GimbalAngleQuatCmd));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = airsim_interfaces__msg__GimbalAngleQuatCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        airsim_interfaces__msg__GimbalAngleQuatCmd__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence__fini(airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      airsim_interfaces__msg__GimbalAngleQuatCmd__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence *
airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence__create(size_t size)
{
  airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence * array = (airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence *)malloc(sizeof(airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence__destroy(airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence * array)
{
  if (array) {
    airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence__fini(array);
  }
  free(array);
}
