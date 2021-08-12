// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from airsim_interfaces:msg/Environment.idl
// generated code does not contain a copyright notice
#include "airsim_interfaces/msg/detail/environment__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
// Member `gravity`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `geo_point`
#include "geographic_msgs/msg/detail/geo_point__functions.h"

bool
airsim_interfaces__msg__Environment__init(airsim_interfaces__msg__Environment * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    airsim_interfaces__msg__Environment__fini(msg);
    return false;
  }
  // position
  if (!geometry_msgs__msg__Vector3__init(&msg->position)) {
    airsim_interfaces__msg__Environment__fini(msg);
    return false;
  }
  // geo_point
  if (!geographic_msgs__msg__GeoPoint__init(&msg->geo_point)) {
    airsim_interfaces__msg__Environment__fini(msg);
    return false;
  }
  // gravity
  if (!geometry_msgs__msg__Vector3__init(&msg->gravity)) {
    airsim_interfaces__msg__Environment__fini(msg);
    return false;
  }
  // air_pressure
  // temperature
  // air_density
  return true;
}

void
airsim_interfaces__msg__Environment__fini(airsim_interfaces__msg__Environment * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // position
  geometry_msgs__msg__Vector3__fini(&msg->position);
  // geo_point
  geographic_msgs__msg__GeoPoint__fini(&msg->geo_point);
  // gravity
  geometry_msgs__msg__Vector3__fini(&msg->gravity);
  // air_pressure
  // temperature
  // air_density
}

airsim_interfaces__msg__Environment *
airsim_interfaces__msg__Environment__create()
{
  airsim_interfaces__msg__Environment * msg = (airsim_interfaces__msg__Environment *)malloc(sizeof(airsim_interfaces__msg__Environment));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(airsim_interfaces__msg__Environment));
  bool success = airsim_interfaces__msg__Environment__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
airsim_interfaces__msg__Environment__destroy(airsim_interfaces__msg__Environment * msg)
{
  if (msg) {
    airsim_interfaces__msg__Environment__fini(msg);
  }
  free(msg);
}


bool
airsim_interfaces__msg__Environment__Sequence__init(airsim_interfaces__msg__Environment__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  airsim_interfaces__msg__Environment * data = NULL;
  if (size) {
    data = (airsim_interfaces__msg__Environment *)calloc(size, sizeof(airsim_interfaces__msg__Environment));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = airsim_interfaces__msg__Environment__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        airsim_interfaces__msg__Environment__fini(&data[i - 1]);
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
airsim_interfaces__msg__Environment__Sequence__fini(airsim_interfaces__msg__Environment__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      airsim_interfaces__msg__Environment__fini(&array->data[i]);
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

airsim_interfaces__msg__Environment__Sequence *
airsim_interfaces__msg__Environment__Sequence__create(size_t size)
{
  airsim_interfaces__msg__Environment__Sequence * array = (airsim_interfaces__msg__Environment__Sequence *)malloc(sizeof(airsim_interfaces__msg__Environment__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = airsim_interfaces__msg__Environment__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
airsim_interfaces__msg__Environment__Sequence__destroy(airsim_interfaces__msg__Environment__Sequence * array)
{
  if (array) {
    airsim_interfaces__msg__Environment__Sequence__fini(array);
  }
  free(array);
}
